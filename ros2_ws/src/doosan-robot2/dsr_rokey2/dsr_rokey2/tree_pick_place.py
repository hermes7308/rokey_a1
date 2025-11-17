
import rclpy
import DR_init

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP   = "Tool Weight"
ROBOT_TOOL  = "GripperDA_v1"

VELOCITY = 60
ACC      = 60

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp

    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID:   {ROBOT_ID}")
    print(f"ROBOT_MODEL:{ROBOT_MODEL}")
    print(f"ROBOT_TCP:  {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY:   {VELOCITY}")
    print(f"ACC:        {ACC}")
    print("#" * 50)


def perform_task():
    from DSR_ROBOT2 import movej, movel, posj, set_digital_output, wait

    safe_dz  = 80.0

    rx = 90.0
    ry = 90.0  #135로 변경하면 사선으로 집음
    rz = 90.0

    vel_approach = 100.0
    acc_approach = 100.0
    vel_work     = 30.0
    acc_work     = 60.0

    def gripper_open():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        wait(0.4)

    def gripper_close():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait(0.4)

    def pick_and_place(px, py, pz, tx, ty, tz):
        pick_down  = [px, py, pz,           rx, ry, rz]
        pick_up    = [px, py, pz + safe_dz, rx, ry, rz]

        place_down = [tx, ty, tz,           rx, ry, rz]
        place_up   = [tx, ty, tz + safe_dz, rx, ry, rz]

        movel(pick_up,   vel=vel_approach, acc=acc_approach)
        movel(pick_down, vel=vel_work,     acc=acc_work)
        gripper_close()
        movel(pick_up,   vel=vel_approach, acc=acc_approach)

        movel(place_up,   vel=vel_approach, acc=acc_approach)
        movel(place_down, vel=vel_work,     acc=acc_work)
        gripper_open()
        movel(place_up,   vel=vel_approach, acc=acc_approach)

    JReady = posj(0, 0, 90, 0, 90, 0)
    print("Move to JReady...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    pick_points = [
        [525.0, -150.0, 80.0],
        [525.0,  -75.0, 80.0],
        [525.0,    0.0, 80.0],
        [600.0, -150.0, 80.0],
        [600.0,  -75.0, 80.0],
        [600.0,    0.0, 80.0],
        [675.0, -150.0, 80.0],
        [675.0,  -75.0, 80.0],
        [675.0,    0.0, 80.0],
    ]

    place_points = [
        [300.0,   0.0, 80.0],
        [300.0, -75.0, 80.0],
        [300.0,-150.0, 80.0],
        [375.0,   0.0, 80.0],
        [375.0, -75.0, 80.0],
        [375.0,-150.0, 80.0],
        [450.0,   0.0, 80.0],
        [450.0, -75.0, 80.0],
        [450.0,-150.0, 80.0],
    ]

    print("Start 3x3 pick & place...")
    gripper_open()

    for i in range(len(pick_points)):
        px, py, pz = pick_points[i]
        tx, ty, tz = place_points[i]
        print(f"[{i+1}/9] pick ({px}, {py}, {pz}) -> place ({tx}, {ty}, {tz})")
        pick_and_place(px, py, pz, tx, ty, tz)

    print("Task finished.")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tree_pick_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
