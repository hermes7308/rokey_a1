import rclpy
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


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트

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
    print("Performing force control task...")
    from DSR_ROBOT2 import (
        release_compliance_ctrl,release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        movej,
        movel,wait,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_BASE,posx,
        posj, get_current_posx, movec
    )
    from math import cos, sin

    P0 = posj(0,0,90,0,90,0)
    movej(P0, vel=20, acc=30)

    # coordinate: example
    p0, _ = get_current_posx()
    # p0[2] -= 414.0
    p0[2] -= 100
    movel(p0, vel=30, acc=100)

    import copy
    r = 100.0

    c = copy.deepcopy(p0)
    c[0] += r

    mp0 = copy.deepcopy(p0)

    loopN = 10

    for i in range(loopN):
        p0[2] -= 10.0
        movel(p0, vel=30, acc=100)
        p0[2] += 10.0
        movel(p0, vel=30, acc=100)
        pre_p0 = p0
        p0[0] = c[0] - r * cos((2.0*3.14159/loopN) * (i+1.0))
        p0[1] = c[1] + r * sin((2.0*3.14159/loopN) * (i+1.0))
        mp0[0] = c[0] - r * cos((2.0*3.14159/loopN) * (i+0.5))
        mp0[1] = c[1] + r * sin((2.0*3.14159/loopN) * (i+0.5))
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