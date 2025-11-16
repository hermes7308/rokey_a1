import signal
import time
import sys
import subprocess
import rclpy
from rclpy.node import Node
from dsr_gss.firebase_utils import get_firebase_db_reference, info


RUNNING = "RUNNING"
STOPPED = "STOPPED"
MAIN_NODE_NAME = "/turtlesim"


class NodeController:
    """ROS2 Node 실행/중단 및 상태확인 기능을 담당하는 클래스"""

    @staticmethod
    def get_pid(node_name: str) -> int | None:
        try:
            result = subprocess.run(
                ["pgrep", "-f", node_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            pids = result.stdout.strip().split("\n")
            return int(pids[0]) if pids and pids[0] else None

        except Exception as e:
            print(f"[ERROR] PID 검색 실패: {e}")
            return None

    @classmethod
    def is_running(cls, node_name: str) -> bool:
        return cls.get_pid(node_name) is not None

    @staticmethod
    def start(node_name: str):
        try:
            subprocess.Popen(
                ["ros2", "run", "turtlesim", "turtlesim_node"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            print(f"[INFO] 노드 시작됨: {node_name}")

        except Exception as e:
            print(f"[ERROR] 노드 시작 실패: {e}")

    @classmethod
    def stop(cls, node_name: str):
        pid = cls.get_pid(node_name)

        if pid is None:
            print(f"[INFO] 노드가 실행 중이 아님: {node_name}")
            return

        try:
            subprocess.run(["kill", "-9", str(pid)])
            print(f"[INFO] 노드 종료됨: {node_name}")

        except Exception as e:
            print(f"[ERROR] 노드 종료 실패: {e}")


class ControlEventManager(Node):
    """Firebase 실시간 이벤트 감시 및 노드 제어 클래스"""

    def __init__(self):
        super().__init__("control_event_manager")
        self.db_ref = get_firebase_db_reference().child("control_event")
        self.status = {
            "current_status": STOPPED,
            "required_status": STOPPED,
        }

        self.start_listener()

    def update_status_from_node(self):
        """실제 노드 상태를 기준으로 DB 데이터 업데이트"""
        node_running = NodeController.is_running(MAIN_NODE_NAME)
        state = RUNNING if node_running else STOPPED

        self.status["current_status"] = state
        self.status["required_status"] = state
        self.db_ref.set(self.status)

    def handle_root_event(self, data):
        """경로 '/' 이벤트 처리"""
        node_running = NodeController.is_running(MAIN_NODE_NAME)
        real_state = RUNNING if node_running else STOPPED

        # DB와 실제 상태 불일치 시 동기화
        if data is None or data.get("current_status") != real_state:
            self.update_status_from_node()

    def on_current_status_changed(self, new_status):
        """Firebase current_status 변경 감지"""
        if new_status == STOPPED and NodeController.is_running(MAIN_NODE_NAME):
            print("[INFO] 노드 이미 실행 중이므로 상태 동기화")
            self.db_ref.child("current_status").set(RUNNING)

    def on_required_status_changed(self, new_status):
        """Firebase required_status 변경 감지 → 노드 실행/종료"""
        if new_status == RUNNING:
            NodeController.start(MAIN_NODE_NAME)
            self.db_ref.child("current_status").set(RUNNING)

        elif new_status == STOPPED:
            NodeController.stop(MAIN_NODE_NAME)
            self.db_ref.child("current_status").set(STOPPED)

    def listener(self, event):
        print(f"\nEvent Type: {event.event_type}")
        print(f"Path: {event.path}")
        print(f"Data: {event.data}\n")

        if event.path == "/":
            self.handle_root_event(event.data)
            return

        if event.path == "/current_status":
            self.on_current_status_changed(event.data)
            return

        if event.path == "/required_status":
            self.on_required_status_changed(event.data)
            return

    def start_listener(self):
        """Firebase 실시간 LISTEN 시작"""
        self.db_ref.listen(self.listener)


def signal_handler(sig, frame):
    print("\n[SIGINT] 종료 시그널 수신됨. 프로그램을 종료합니다.")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def main(args=None):
    # 1. rclpy 초기화
    rclpy.init(args=args)

    # 2. ControlEventListener 노드 생성
    node = ControlEventManager()

    try:
        info("ControlEventManager 를 실행합니다.")
        # 3. spin() 함수로 노드 실행 및 유지
        # 이 함수는 메시지가 오기를 기다리며 콜백을 처리합니다.
        # Ctrl+C (SIGINT)를 받으면 자동으로 블록이 풀리고 다음 코드로 넘어갑니다.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Ctrl+C가 눌렸을 때, rclpy.spin()이 종료되고 이 예외가 발생합니다.
        node.get_logger().info(
            "❌ Node shutting down gracefully due to Keyboard Interrupt."
        )

    finally:
        # 4. 노드 정리 (try-finally 구문으로 안정적인 정리 보장)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
