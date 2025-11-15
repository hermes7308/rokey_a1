import rclpy
import DR_init
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dsr_gss.firebase_utils import get_firebase_db_reference


# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 100
ACC = 100

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def perform_task():
    global test_publisher
    # 코드를 작성하시오.
    """로봇이 주기적으로 작업을 수행하도록 설정"""
    from DSR_ROBOT2 import wait
    while True:
        # print("Hello, Robot!")
        test_publisher.publish(String(data="Hello, Pub Robot!"))
        wait(1.0)  # 1초 대기


def listener_callback(msg):
    # from DSR_ROBOT2 import fkin
    global test_publisher

    test_publisher.publish(String(data="Hello, Sub Robot!"))
    # data = [pos for pos in msg.position]
    # db_ref.child("dsr_gss").child("joint_coordinate").set(data)
    # node.get_logger().info(f"Uploaded joint coordinates: {data}")
    # node.get_logger().info(f"Uploaded task coordinates: {fkin(data)}")


def main(args=None):
    global node
    global db_ref
    global robot_pose_uploader
    global test_publisher

    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("robot_status_uploader", namespace=ROBOT_ID)
    db_ref = get_firebase_db_reference()

    robot_pose_uploader = node.create_subscription(
        JointState, "/dsr01/joint_states", listener_callback, 10
    )
    robot_pose_uploader

    test_publisher = node.create_publisher(String, "test_joint_states", 10)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
