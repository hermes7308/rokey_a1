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

    # ===== 필요한 Doosan API 불러오기 =====
    from DSR_ROBOT2 import (
        posx, posj,
        movej, amovej,    # 조인트 이동
        movel, amovel,    # Cartesian 이동
        DR_MV_MOD_REL,    # 상대 이동 모드
        DR_MV_MOD_ABS,    # 절대 이동 모드
        set_digital_output,
        ON, OFF,
        DR_TOOL,          # 공구 좌표계
        DR_BASE,          # 기본(Base) 좌표계
        get_current_posx, # 현재 좌표 읽기
        wait
    )

    # ==========================
    #  땅 파기용 전용 동작 함수들
    # ==========================

    # ■ 삽을 아래로 밀어넣는 동작
    def dig():
        movel([0, 35, 0, 0, 0, 0],
              v=VELOCITY, a=ACC,
              ref=DR_TOOL, mod=DR_MV_MOD_REL)

    # ■ 삽 파기 자세(굴절 + 회전)로 변형
    def change_formation():
        # End-Effector 회전 (기울이기)
        amovel([0, 0, 0, 90, 45, 0],
               v=VELOCITY, a=ACC,
               ref=DR_BASE, mod=DR_MV_MOD_REL)

        # Tool 자체 회전 (yaw 회전)
        movej([0, 0, 0, 0, 0, 90],
              v=VELOCITY, a=ACC,
              mod=DR_MV_MOD_REL)

    # ■ Z축으로 내려가기
    def down():
        movel([0, 0, -68, 0, 0, 0],
              v=VELOCITY, a=ACC,
              ref=DR_BASE, mod=DR_MV_MOD_REL)

    # ■ Z축으로 올라오기
    def up():
        movel([0, 0, 68, 0, 0, 0],
              v=VELOCITY, a=ACC,
              ref=DR_BASE, mod=DR_MV_MOD_REL)

    # ■ 절대좌표로 목표 땅 파기 위치 이동
    def move_target(pos, v=VELOCITY, a=ACC,
                    ref=DR_BASE, mod=DR_MV_MOD_ABS):
        movel(pos, v=v, a=a, ref=ref, mod=mod)

    # ■ Tool Box(삽 위치)로 이동하는 경유점 경로
    def move_toolbox():
        mid_point1 = posj([-92.55, 29.43, 59.55, 1.13, 85.2, -89])
        mid_point2 = posj([-88.69, 29.25, 64.38, -94.96, 92.09, -88.99])
        toolbox    = posx([-491.03, -352.87, 222.27, 91.40, 86.8, 90.18])

        # 안정적인 접근을 위해 두 개의 중간자세 사용
        movej(mid_point1, v=VELOCITY, a=ACC, mod=DR_MV_MOD_ABS)
        movej(mid_point2, v=VELOCITY, a=ACC, mod=DR_MV_MOD_ABS)
        movel(toolbox,  v=VELOCITY, a=ACC,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

    # ■ 삽 잡기
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    # ■ 삽 놓기
    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    # ■ 기본 자세로 이동
    def home():
        j_ready = [0, 0, 90, 0, 90, 0]
        movej(j_ready, v=30, a=ACC)

    # ■ get_current_posx 반환값을 통일된 리스트로 변환
    def to_pos_list(p):
        if isinstance(p, dict):
            return list(p["posx"])
        if isinstance(p, (list, tuple)):
            if len(p) >= 6 and isinstance(p[0], (int, float)):
                return list(p[:6])
            if len(p) >= 1 and isinstance(p[0], (list, tuple)):
                return list(p[0][:6])
        raise RuntimeError(f"get_current_posx() 형식 이상: {p}")

    # ■ “한 지점에서 땅 파고, 옮기고, 버리기” 전체 동작 패키지
    def whole_digging():

        # 1) 삽을 내리고
        down()

        # 2) 현재 위치 저장 (절대 복귀용)
        curr1 = to_pos_list(get_current_posx())

        # 3) 땅 파기
        dig()

        # 4) 파고난 후 깊이·자세 조정한 새 좌표 생성
        curr2 = to_pos_list(get_current_posx())
        curr2[1] -= 100     # y축 뒤로 이동
        curr2[2] -= 78      # z축 아래 조정
        curr2[3], curr2[4], curr2[5] = 90, 180, 90

        # 5) 새 위치로 이동 (절대)
        movel(curr2, v=VELOCITY, a=ACC,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

        # 6) 땅에서 빼내면서 앞으로 이동
        movel([0, 50, 0, 0, 0, 0],  # 상대 +y
              v=VELOCITY, a=ACC,
              ref=DR_BASE, mod=DR_MV_MOD_REL)

        # 7) 위로 빼기
        up()

        # 8) 버리는 장소 = home()로 복귀
        home()

        # 9) 삽 털기 위한 회전
        movej([-90, 0, 0, 0, 0, 0],
              v=VELOCITY, a=ACC,
              mod=DR_MV_MOD_REL)

        movej([0, 0, 0, 0, -90, 0],
              v=VELOCITY, a=ACC,
              mod=DR_MV_MOD_REL)

        # 10) 다시 기본자세로
        home()

    # ======================================
    #  메인 동작 시퀀스
    # ======================================

    # ■ 두 군데 땅 파기 지점 지정
    target1 = posx([305, -195, 0, 0, 0, 0])
    target2 = posx([5, 120, 0, 0, 0, 0])
    target_list = [target1, target2]

    # 시작: 기본자세 + 삽 해제
    home()
    release()

    # Step 1) ToolBox에서 삽 들기
    move_toolbox()
    movel([0, 60, 0, 0, 0, 0],
          v=VELOCITY, a=ACC,
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    grip()
    wait(1)
    up()
    wait(1)

    # Step 2) 기본자세 복귀 후 땅 파기 시작
    home()

    for tgt in target_list:
        change_formation()                       # 땅 파기용 자세 변형
        move_target(tgt, v=VELOCITY, a=ACC,
                    ref=DR_BASE, mod=DR_MV_MOD_REL)
        whole_digging()                          # 전체 파기 동작 실행

    # Step 3) 작업 종료 → 삽 반납
    home()
    move_toolbox()
    up()
    movel([0, 60, 0, 0, 0, 0],
          v=VELOCITY, a=ACC,
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    movel([0, 0, -65, 0, 0, 0],
          v=VELOCITY, a=ACC,
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    release()
    wait(1)
    movel([0, -60, 0, 0, 0, 0],
          v=VELOCITY, a=ACC,
          ref=DR_BASE, mod=DR_MV_MOD_REL)

    # Step 4) 홈 포지션으로 완료
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