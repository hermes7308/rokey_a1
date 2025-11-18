import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"  # 본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  # 본인 그리퍼 이름 설정

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
    print("#" * 50)


def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    from DSR_ROBOT2 import (
        ON,
        OFF,
        DR_MV_MOD_REL,
        set_digital_output,
        wait,
        posj,
        posx,
        movej,
        movel,
    )
    from dsr_gss.firebase_utils import (
        debug,
        info,
        warning,
        error,
        increase_planted_count,
    )

    # === Method 정의 부분 ==================================================
    def move_to_home():
        movej(posj(0, 0, 90, 0, 90, 0), acc=ACC, vel=VELOCITY)

    def open_gripper():
        # 컨트롤러 DO + 툴 DO
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.4)

    def close_gripper():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.4)

    # === Step scenario 정의 부분 ===========================================
    def step1():
        # TODO: 강종아
        pass

    def step2():
        # ===== 정의 =====
        safe_dz = 125.0  # 위로 들어올리는 높이
        rx = 180.0
        ry = -150.0
        rz = 90.0

        vel_approach = 100.0
        acc_approach = 100.0
        vel_work = 30.0
        acc_work = 60.0

        pz = 250.0  # z_pick
        tz = 250.0  # z_place

        pick_points = [
            # (255.0, -202.0),  # 1번
            (370.0, -202.0),  # 2번
            # (370.0, -72.0),  # 3번
        ]

        place_points = [
            # (256.24, 161.58),  # 1번
            # (556.24, 161.58),  # 2번
            (556.24, -138.42),  # 3번
        ]

        if len(pick_points) != len(place_points):
            error("Pick/Place 포인트의 갯수가 일치하지 않습니다. 작업을 건너뜁니다.")
            return

        n = len(pick_points)

        # ===== 동작 =====
        info("===== 묘목 심기 시작 =====")
        move_to_home()
        for i in range(n):
            px, py = pick_points[i]
            tx, ty = place_points[i]
            info(f"[{i+1}/{n}] pick ({px}, {py}, {pz}) -> place ({tx}, {ty}, {tz})")
            space_gap = 100

            first_pick_up = posx(px - space_gap, py, pz + safe_dz, rx, ry, rz)
            first_place_up = posx(tx, ty, tz + safe_dz, rx, ry, rz)

            # ---- 접근 & 집기 ----
            open_gripper()
            movel(first_pick_up, vel=vel_approach, acc=acc_approach)
            movel(
                posx(space_gap, 0, -120, 0, 0, 0),
                mod=DR_MV_MOD_REL,
                vel=vel_work,
                acc=acc_work,
            )
            close_gripper()
            movel(
                posx(0, 0, 200, 0, 0, 0), mod=DR_MV_MOD_REL, vel=vel_work, acc=acc_work
            )

            # ---- 이동 & 놓기 ----
            movel(first_place_up, vel=vel_approach, acc=acc_approach)
            movel(
                posx(0, 0, -100, 0, 0, 0), mod=DR_MV_MOD_REL, vel=vel_work, acc=acc_work
            )
            open_gripper()
            movel(
                posx(0, 0, 200, 0, 0, 0), mod=DR_MV_MOD_REL, vel=vel_work, acc=acc_work
            )
            movel(
                posx(-space_gap, 0, 0, 0, 0, 0),
                mod=DR_MV_MOD_REL,
                vel=vel_work,
                acc=acc_work,
            )

        move_to_home()
        # 심은 묘목 갯수 증가
        increase_planted_count()
        info("===== 묘목 심기 종료 =====")

    def step3():
        # TODO: 김재우
        pass

    def step4():
        # TODO: 황규빈
        pass

    # === Play 부분 =========================================================
    # 초기 위치 home 으로 이동
    move_to_home()

    # Steps
    step1()
    step2()
    step3()
    step4()

    # 작업 마무리 하고 home 으로 이동
    move_to_home()


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

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
