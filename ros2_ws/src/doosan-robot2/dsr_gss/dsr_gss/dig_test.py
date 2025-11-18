import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" #본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  #본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 60
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
    """로봇이 수행할 작업"""
    print("Performing task...")
    from DSR_ROBOT2 import posx,movej,movel,movejx,DR_MV_MOD_REL,DR_MV_MOD_ABS,set_digital_output,ON,OFF,DR_TOOL,get_current_posx,wait,DR_BASE      # 필요한 기능만 임포트

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)

    def down():
        movel([0, 0, -50, 0, 0, 0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
    
    def up():
        movel([0, 0, 50, 0, 0, 0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)

    def home():
        # 초기 위치 및 목표 위치 설정
        JReady = ([0, 0, 90, 0, 90, 0])
        movej(JReady, v = VELOCITY, a = ACC)

    def dig():
        digging = [0, 30, 0, 0, 0, 0]
        movel(digging, v = VELOCITY, a = ACC, ref = DR_TOOL, mod = DR_MV_MOD_REL)

    def move_to_dig():
        digging_part = [281.9,-280.89,336.84,87.5,-149.86,88.73]
        movel(digging_part, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_ABS)
    
    def tilt_down():
        movel([0, 0, 0, -45, 0, 0], v=VELOCITY, a=ACC, mod=DR_MV_MOD_REL)

    def tilt_up():
        movel([0, 0, 0, 45, 0, 0], v=VELOCITY, a=ACC, mod=DR_MV_MOD_REL)

    # start_x = -75
    # start_y = -200
    # start_z = 0
    # start_pose = posx([start_x, start_y, start_z, 90, 180, 90])

    # dy = 100
    # line_dy = -450

    # holes_line1 = [
    #     posx([start_x, start_y + dy*i, start_z, 90, 180, 90])
    #     for i in range(5)
    #     ]

    # holes_line2 = [
    #     posx([start_x, start_y + line_dy + dy*i, start_z, 90, 180, 90])
    #     for i in range(5)
    #     ]


    home()
    release()
    # 툴 박스 이동 및 삽 집기 위한 모션 제어
    move_to_tool_box = [500, -200, 200, 90, 90, 90]

    movel(move_to_tool_box, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
    # movel(move_parallel, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
    down()
    grip()
    up()
    home()
    ######################## 
    move_to_dig()  # 차후에 묘목 심는 위치로 이동해야함 첫번째 라인
    ########################
    # 땅 파는 모션
    movej([0, 0, 0, 45, 0, 0], v = VELOCITY, a = ACC, mod = DR_MV_MOD_REL)
    # 땅 파기 전반적인 모션
    def all_digging():
        for i in range(5):
            move_forward = [0, 100, 0, 0, 0, 0]
        # movel(p, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
            a = get_current_posx()
            if isinstance(a, tuple):
                curr = list(a[0])   # posx 부분
            else:
                curr = list(a)
            # 파기
            dig()
            # 뜨기
            movej([0,0,0,45,0,0], v = VELOCITY, a = ACC, mod = DR_MV_MOD_REL)
            # 들어올리기
            up()
            # 옆으로 이동
            movel([100,0,0,0,0,0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)
            # 붓기
            movej([0,0,0,0,0,90], v = VELOCITY, a = ACC, mod = DR_MV_MOD_REL)
            movel(curr, v = VELOCITY, a = ACC)
        # 다음 위치로 이동
            movel(move_forward, v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)

    # for p in holes_line1:
    all_digging()

    # # 다음 라인으로 이동
    movel([-100,-450,0,0,0,0], v = VELOCITY, a = ACC, ref = DR_BASE, mod = DR_MV_MOD_REL)

    # for p in holes_line2:
    all_digging()

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("dig_test", namespace=ROBOT_ID)

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