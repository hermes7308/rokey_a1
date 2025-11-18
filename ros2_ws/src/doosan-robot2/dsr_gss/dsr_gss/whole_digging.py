import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" #본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  #본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 100
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # Tool과 TCP 설정
    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    # 설정된 설정값 출력
    # print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#"*50)


def perform_task():
    print("Performing task...")
    from DSR_ROBOT2 import (posx, posj,
                            movej,
                            movel,
                            DR_MV_MOD_REL,
                            DR_MV_MOD_ABS,
                            set_digital_output,
                            ON, OFF, DR_TOOL,
                            DR_BASE,
                            get_current_posx, wait, movec, movejx
                            )
    
#-------------------------------------------------------
    def dig():
        movel([0, 35, 0, 0, 0, 0], v = VELOCITY, a = ACC, ref = DR_TOOL, mod = DR_MV_MOD_REL)
#-------------------------------------------------------
    def change_formation():
        movel([0, 0, 0, 90, 45, 0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movej([0, 0, 0, 0, 0, 90], v = VELOCITY, a = ACC, mod = DR_MV_MOD_REL)
#-------------------------------------------------------
    def down():
        movel([0,0,-70,0,0,0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
#-------------------------------------------------------
    def up():
        movel([0,0,70,0,0,0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
#-------------------------------------------------------
    def move_target(pos, v=VELOCITY, a=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS):
        movel(pos, v=v, a=a, ref=ref, mod=mod)
#-------------------------------------------------------
    def move_toolbox():
    # 초기 위치 및 목표 위치 설정
        # movel([0, 0, 0, 0, 90, 0], v = VELOCITY, a = ACC, mod = DR_MV_MOD_REL)
        mid_point1 = posj([-92.55, 29.43, 59.55, 1.13, 85.2, -89])
        mid_point2 = posj([-88.69, 29.25, 64.38, -94.96, 92.09, -88.99])
        toolbox = posx([-491.03, -352.87, 222.27, 91.40, 86.8, 90.18])
        movej(mid_point1, v = VELOCITY, a = ACC, mod = DR_MV_MOD_ABS)
        movej(mid_point2, v = VELOCITY, a = ACC, mod = DR_MV_MOD_ABS)
        movel(toolbox, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_ABS)
 
#-------------------------------------------------------
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
#-------------------------------------------------------
    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
#-------------------------------------------------------
    def home():
        JReady = ([0, 0, 90, 0, 90, 0])
        movej(JReady, v = 30, a = ACC)
#-------------------------------------------------------
    def change_formation():
        movel([0, 0, 0, 90, 45, 0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movej([0, 0, 0, 0, 0, 90], v = VELOCITY, a = ACC, mod = DR_MV_MOD_REL)
#-------------------------------------------------------
    def whole_digging():

        down()

        a = get_current_posx()

        if isinstance(a, dict):
            curr1 = a['posx']

        elif isinstance(a, (list, tuple)):
            if len(a) >= 6 and isinstance(a[0], (int, float)):
                curr1 = list(a[:6])
            elif len(a) >= 1 and isinstance(a[0], (list, tuple)):
                curr1= list(a[0][:6])
            else:
                raise RuntimeError(f"get_current_posx() 반환 형식을 못 알아먹겠음: {a}")
        else:
            raise RuntimeError(f"get_current_posx() 타입이 이상함: {type(a)}, {a}")

        dig()

        b = get_current_posx()

        if isinstance(b, dict):
            curr2 = b['posx']

            # 2) 튜플/리스트 형태인 경우
        elif isinstance(b, (list, tuple)):
                # a가 [x,y,z,rx,ry,rz] 이런 1차원 리스트면
            if len(b) >= 6 and isinstance(b[0], (int, float)):
                curr2 = list(b[:6])
                # a가 ([x,y,z,rx,ry,rz], ref) 이런 2중 구조면
            elif len(b) >= 1 and isinstance(b[0], (list, tuple)):
                    curr2 = list(b[0][:6])
            else:
                raise RuntimeError(f"get_current_posx() 반환 형식을 못 알아먹겠음: {a}")
        else:
            raise RuntimeError(f"get_current_posx() 타입이 이상함: {type(a)}, {a}")
        curr2[1] = curr2[1] - 100

        curr2[2] = curr2[2] - 80

        curr2[3], curr2[4], curr2[5] = 90, 180, 90

        movel(curr2, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_ABS)

        print(">>> Start first movel")
        # movel([0, 0, 0, 0, 0, 0], v=VELOCITY, a=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel([0, 50, 0, 0, 0, 0], v=VELOCITY, a=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        print(">>> Finished first movel")

        print(">>> Calling up()")
        up()
        print(">>> Returned from up()")

        print(">>> Starting second movel")
        # 버리는 곳 위치 정해서 절대 좌표로 진행
        # movel([-30, 0, 0, 0, 0, 0], v=VELOCITY, a=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        home()
        print(">>> Finished second movel")

        print(">>> Final movel rotation")
        movel([0, 0, 0, 0, -45, 0], v=VELOCITY, a=ACC, mod=DR_MV_MOD_REL)
        print(">>> Task done")


        movel(curr1, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_ABS)

        up()
#-------------------------------------------------------

    target1 = posx([305, -190, 0, 0, 0, 0])
    target2 = posx([0, 200, 0, 0, 0, 0])
    target3 = posx([-300, 0, 0, 0, 0, 0])
    target = [target1, target2, target3]
    
    # home()

    # release()
    # move_toolbox()

    # movel([0, 60, 0, 0, 0, 0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
    # grip()
    # wait(1)
    # up()
    # wait(1)
    
    home()
    
    change_formation()
    for i in range(len(target)):
        move_target(target[i], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
        whole_digging()

    home()

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("whole_digging", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()