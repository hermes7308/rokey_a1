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
        set_ref_coord, task_compliance_ctrl, set_desired_force, DR_FC_MOD_REL, DR_MV_MOD_ABS,
        check_force_condition, DR_AXIS_Z, DR_BASE, DR_MV_MOD_REL
    )

    # # home 위치로 돌아가는 작업
    # # 경우에 따라 삭제 가능
    j0 = posj(0,0,90,0,90,0)
    movej(j0, vel=60, acc=30)
    movel([0, 0, 100, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)

    movej([-180, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL, vel=60, acc=20)

    ## gripping 동작 추가
    print("movej 끝")
    tool_pos = posx(-447.19, 9.11, 478.33, 0.0, 180.0, 0.0) # 잡을 위치
    tool_up = copy.deepcopy(tool_pos)
    tool_up[2] += 50.0
    print("tool_up 도착")
    movel(tool_up, vel=60, acc=100, mod=DR_MV_MOD_ABS)
    # gripper 열기
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3)
    
    movel(tool_pos, vel=60, acc=100, mod=DR_MV_MOD_ABS)
    print("tool_pos 도착")
    # wait(2)
    # gripper 닫기
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(3)
    movel([0, 0, 100, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)
    movel([100, 0, 0, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)
    # movel(tool_up, vel=60, acc=100)
    wait(1)

    movej([180, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL, vel=60, acc=20)

    # 나무 위치들
    treeLocations = [posx(356.24, 161.58, 340.0, 0.0, 180.0, 0.0),
                     posx(656.24, 161.58, 340.0, 0.0, 180.0, 0.0),
                     posx(656.24, 161.58-300.0, 340.0, 0.0, 180.0, 0.0)]

    r = 80.0
    loopN = 4

    dummy_index = 0
    for treeLocation in treeLocations:
        if dummy_index == 0:
            c = posx(treeLocation)
            p0 = copy.deepcopy(c)
            p0[1] -= r
            movel(p0, vel=30, acc=100)

            mp0 = copy.deepcopy(p0)

            for i in range(loopN):
                p0[2] -= 10.0
                movel(p0, vel=60, acc=100)
                p0[2] += 10.0
                movel(p0, vel=60, acc=100)
                p0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+1.0) + (3.0/2.0)*pi)
                p0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+1.0) + (3.0/2.0)*pi)
                mp0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+0.5) + (3.0/2.0)*pi)
                mp0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+0.5) + (3.0/2.0)*pi)
                movec(mp0,p0,vel=60,acc=100)
        elif dummy_index == 1:
            c = posx(treeLocation)
            p0 = copy.deepcopy(c)
            p0[1] += r
            movel(p0, vel=30, acc=100)

            mp0 = copy.deepcopy(p0)

            for i in range(loopN - 1):
                p0[2] -= 10.0
                movel(p0, vel=60, acc=100)
                p0[2] += 10.0
                movel(p0, vel=60, acc=100)
                p0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+1.0) + (1.0/2.0)*pi)
                p0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+1.0) + (1.0/2.0)*pi)
                mp0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+0.5) + (1.0/2.0)*pi)
                mp0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+0.5) + (1.0/2.0)*pi)
                movec(mp0,p0,vel=60,acc=100)
        else:
            c = posx(treeLocation)
            p0 = copy.deepcopy(c)
            p0[1] += r
            movel(p0, vel=30, acc=100)

            mp0 = copy.deepcopy(p0)

            for i in range(loopN):
                p0[2] -= 10.0
                movel(p0, vel=60, acc=100)
                p0[2] += 10.0
                movel(p0, vel=60, acc=100)
                p0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+1.0) + (1.0/2.0)*pi)
                p0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+1.0) + (1.0/2.0)*pi)
                mp0[0] = c[0] + r * cos((2.0*pi/loopN) * (i+0.5) + (1.0/2.0)*pi)
                mp0[1] = c[1] + r * sin((2.0*pi/loopN) * (i+0.5) + (1.0/2.0)*pi)
                movec(mp0,p0,vel=60,acc=100)

        dummy_index += 1

    ## tool 복귀
    # 충돌 대비해서 목표 위치보다 수직으로 위인 지점으로 선형 이송 시키기
    ptb1, _ = get_current_posx()
    ptb1[2] += 100.0
    movel(ptb1, vel=60, acc=100)
    movej(j0, vel=60, acc=30)
    movel([0, 0, 100, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)

    movej([-180, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL, vel=60, acc=20)

    # tool 꽂을 곳 위로 이동
    movel(tool_up, vel=60, acc=100)
    # tool 꽂기
    # 일단 힘 제어 안  쓰고
    movel(tool_pos, vel=10, acc=100)
    # set_ref_coord(1) # Tool 좌표계 설정
    # task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
    # wait(0.5) # 안정화 대기(필수)
    # set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    # while check_force_condition(DR_AXIS_Z, min=10):
    #     print("Tool 삽입중")
    #     wait(0.5)
    ##
    # gripper 열기
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(1)
    # 좌표계 복귀
    set_ref_coord(DR_BASE)
    # 충돌 대비해서 위로 올리기
    pc, _ = get_current_posx()
    pc[2] += 200.0
    movel(pc, vel=60, acc=100)
    # home으로 돌아오기
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