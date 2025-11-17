#tamping.py
import rclpy
from rclpy.executors import MultiThreadedExecutor
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

import copy
from math import cos, sin, pi

from dsr_interfaces.srv import TreeLocation

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    print("[initialize_robot] DR_init.__dsr__node:", DR_init.__dsr__node)
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print(f"MODE: {ROBOT_MODE_AUTONOMOUS}")
    print("#" * 50)


def perform_task():
    """로봇이 수행할 작업"""
    print("작업 시작")
    from DSR_ROBOT2 import (
        movej,
        movel,
        posj, get_current_posx, movec, posx, set_digital_output, wait, ON, OFF,
        set_ref_coord, task_compliance_ctrl, set_desired_force, DR_FC_MOD_REL,
        check_force_condition, DR_AXIS_Z, DR_BASE
    )

    # # home 위치로 돌아가는 작업
    # # 경우에 따라 삭제 가능
    j0 = posj(0,0,90,0,90,0)
    movej(j0, vel=20, acc=30)

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("tamping", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        perform_task()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()