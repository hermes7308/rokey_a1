import rclpy
import threading
import time
import DR_init

from sensor_msgs.msg import JointState
from dsr_gss.firebase_utils import get_firebase_db_reference, info


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY = 40
ACC = 60

SLEEP_TIME = 1


def initialize_robot():
    """Tool, TCP 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp

    print("로봇 초기 설정 수행 중...")
    print(f"Tool = {ROBOT_TOOL}, TCP = {ROBOT_TCP}")

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def perform_task():
    """로봇 작업 수행"""
    from DSR_ROBOT2 import posx, posj, movej, movel

    db = get_firebase_db_reference()
    print("로봇 작업을 시작합니다.")
    info("MoveActionManager 시작합니다.")
    while True:
        action = db.child("control_event/action").get()
        if action["status"].lower() != "ready":
            time.sleep(SLEEP_TIME)
            continue

        db.child("control_event/action/status").set("RUNNING")

        if action["actionType"].lower() == "movej":
            print("MoveJ")
            pos = posj(
                [
                    float(action["data"]["J1"]),
                    float(action["data"]["J2"]),
                    float(action["data"]["J3"]),
                    float(action["data"]["J4"]),
                    float(action["data"]["J5"]),
                    float(action["data"]["J6"]),
                ]
            )
            acc = float(action["data"]["Acceleration"])
            vel = float(action["data"]["Velocity"])
            movej(pos, vel, acc)
            db.child("control_event/action/status").set("DONE")

        if action["actionType"].lower() == "movel":
            print("MoveL")
            pos = posx(
                [
                    float(action["data"]["X"]),
                    float(action["data"]["Y"]),
                    float(action["data"]["Z"]),
                    float(action["data"]["A"]),
                    float(action["data"]["B"]),
                    float(action["data"]["C"]),
                ]
            )
            acc = float(action["data"]["Acceleration"])
            vel = float(action["data"]["Velocity"])
            movel(pos, vel, acc)
            db.child("control_event/action/status").set("DONE")

        time.sleep(SLEEP_TIME)  # 동작 사이에 약간의 대기 시간 추가


def main(args=None):
    global latest_joint

    rclpy.init(args=args)

    node = rclpy.create_node("move_action_manager", namespace=ROBOT_ID)

    # Doosan 노드 등록
    DR_init.__dsr__node = node

    # 초기화
    initialize_robot()

    print("로봇 동작 시작...")

    perform_task()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
