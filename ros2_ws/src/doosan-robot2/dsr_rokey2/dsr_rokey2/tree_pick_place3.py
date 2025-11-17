import rclpy
import DR_init

# ===== 로봇 설정 =====
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP   = "Tool Weight"     # 실제 TCP 이름
ROBOT_TOOL  = "GripperDA_v1"    # 실제 Tool 이름

VELOCITY_J = 60.0
ACC_J      = 30.0

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """Tool / TCP 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp

    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    print("=== Initialize Robot ===")
    print(f"ROBOT_ID   : {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP  : {ROBOT_TCP}")
    print(f"ROBOT_TOOL : {ROBOT_TOOL}")
    print("========================")


def perform_task():
    """DRL 스크립트 내용을 ROS2에서 그대로 실행"""

    from DSR_ROBOT2 import (
        movej, movel, posj, posx,
        set_digital_output, set_tool_digital_output,
        wait,
    )

    # ===== 시작 자세 =====
    J0 = posj(0, 0, 90, 0, 90, 0)
    movej(J0, vel=VELOCITY_J, acc=ACC_J)

    # ===== 기본 설정 =====
    safe_dz  = 125.0    # 위로 들어올리는 높이

    rx = 90.0
    ry = 120.0
    rz = 90.0

    vel_approach = 100.0
    acc_approach = 100.0
    vel_work     = 30.0
    acc_work     = 60.0

    # ===== 그리퍼 설정 =====
    def gripper_open(): 
        # 컨트롤러 DO + 툴 DO
        set_digital_output(1, 0)
        set_digital_output(2, 1)

        set_tool_digital_output(1, 0)
        set_tool_digital_output(2, 1)

        wait(0.4)

    def gripper_close():
        set_digital_output(1, 1)
        set_digital_output(2, 0)

        set_tool_digital_output(1, 1)
        set_tool_digital_output(2, 0)

        wait(0.4)

    # ===== 한 점에 대한 pick & place =====
    def pick_and_place(px, py, pz, tx, ty, tz):

        pick_down  = posx(px, py, pz,           rx, ry, rz)
        pick_up    = posx(px, py, pz + safe_dz, rx, ry, rz)

        place_down = posx(tx, ty, tz,           rx, ry, rz)
        place_up   = posx(tx, ty, tz + safe_dz, rx, ry, rz)

        # ---- 접근 & 집기 ----
        movel(pick_up,   vel=vel_approach, acc=acc_approach)
        movel(pick_down, vel=vel_work,     acc=acc_work)
        gripper_close()
        movel(pick_up,   vel=vel_approach, acc=acc_approach)

        # ---- 이동 & 놓기 ----
        movel(place_up,   vel=vel_approach, acc=acc_approach)
        movel(place_down, vel=vel_work,     acc=acc_work)
        gripper_open()
        movel(place_up,   vel=vel_approach, acc=acc_approach)

    # ===== 메인 루틴 내용 =====
    gripper_open()

    # 3개 pick / place 좌표
    pick_points = [
        (225.0, -225.0),   # 1번
        (300.0, -225.0),   # 2번
        (375.0, -225.0),   # 3번
    ]

    place_points = [
        (650.0, -250.0),   # 1번
        (650.0,   50.0),   # 2번
        (375.0,   50.0),   # 3번
    ]

    z_pick  = 160.0
    z_place = 160.0

    for i in range(3):
        px, py = pick_points[i]
        tx, ty = place_points[i]
        print(f"[{i+1}/3] pick ({px}, {py}, {z_pick}) -> place ({tx}, {ty}, {z_place})")
        pick_and_place(px, py, z_pick, tx, ty, z_place)

    print("=== Task finished ===")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tree_pick_place_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
