import rclpy
import threading
import time
import DR_init

from sensor_msgs.msg import JointState


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY = 40
ACC = 60

# joint state 저장 변수
latest_joint = None


def joint_callback(msg):
    """joint_states 콜백: 데이터 저장만 수행"""
    global latest_joint
    latest_joint = msg.position     # 또는 msg.velocity, msg.effort
    print(f"현재 관절 값: {latest_joint}")  # 필요시 활성화


def sub_thread(node):
    """subscriber를 위한 별도 스레드"""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()   # DSR_ROBOT2 내부 spin과 충돌 없음


def initialize_robot():
    """Tool, TCP 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp

    print("로봇 초기 설정 수행 중...")
    print(f"Tool = {ROBOT_TOOL}, TCP = {ROBOT_TCP}")

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def perform_task():
    """로봇 작업 수행"""
    global latest_joint

    print("주기적 움직임 작업 수행 중...")
    from DSR_ROBOT2 import movej, DR_TOOL, mwait

    JReady = [0, 0, 90, 0, 90, 0]

    print("초기 위치로 이동 중...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    while True:
        print("주기적 움직임 시작...")
        
        movej([0, 0, 50, 0, 90, 0], vel=VELOCITY, acc=ACC)
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        
        time.sleep(2)  # 동작 사이에 약간의 대기 시간 추가

  
def main(args=None):
    global latest_joint

    rclpy.init(args=args)

    node = rclpy.create_node("topic_test", namespace=ROBOT_ID)

    # joint state 구독
    node.create_subscription(
        JointState,
        "/dsr01/joint_states",   # 절대 경로
        joint_callback,
        10
    )

    # subscriber 스레드 실행
    t = threading.Thread(target=sub_thread, args=(node,), daemon=True)
    t.start()

    # Doosan 노드 등록
    DR_init.__dsr__node = node

    # 초기화
    initialize_robot()

    print("로봇 동작 시작...")

    perform_task()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
