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
        posj, get_current_posx, movec, posx, set_digital_output, wait, ON, OFF
    )

    # home 위치로 돌아가는 작업
    # 경우에 따라 삭제 가능
    P0 = posj(0,0,90,0,90,0)
    movej(P0, vel=20, acc=30)

    ## gripping 동작 추가
    pt = posx(450.0, 200.0, 100.0, 0.0, 180.0, 0.0) # 잡을 위치
    pt1 = copy.deepcopy(pt)
    pt1[2] += 30.0
    movel(pt1, vel=30, acc=100)
    # gripper 열기
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1)
    movel(pt, vel=30, acc=100)
    # gripper 닫기
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)
    wait(1)
    movel(pt1, vel=30, acc=100)

    # 나무 위치
    treeLocation = posx(300.0, -300.0, 50.0, 0.0, 180.0, 0.0)

    r = 100.0

    c, _ = get_current_posx() # 자료형 문제
    for i in range(6):
        c[i] = treeLocation[i]
    p0 = copy.deepcopy(c)
    p0[1] -= r
    movel(p0, vel=30, acc=100)

    mp0 = copy.deepcopy(p0)

    loopN = 10

    for i in range(loopN):
        p0[2] -= 10.0
        movel(p0, vel=30, acc=100)
        p0[2] += 10.0
        movel(p0, vel=30, acc=100)
        p0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+1.0) + (3.0/2.0)*pi)
        p0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+1.0) + (3.0/2.0)*pi)
        mp0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+0.5) + (3.0/2.0)*pi)
        mp0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+0.5) + (3.0/2.0)*pi)
        movec(mp0,p0,vel=30,acc=100)

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