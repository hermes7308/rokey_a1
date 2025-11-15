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


def perform_task(treeLocation: list):
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

    r = 100.0

    c = posx(treeLocation)
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

    # 로봇 초기화
    initialize_robot()

    node.get_logger().info(
        f"Initializing robot: id={ROBOT_ID}, model={ROBOT_MODEL}, "
        f"tool={ROBOT_TOOL}, tcp={ROBOT_TCP}"
    )

    pending_tree_location = {"data": None}

    # 서비스 콜백 함수 (Node를 상속하지 않고 클로저로 묶기)
    def handle_tamping_task(request: TreeLocation.Request, response: TreeLocation.Response):
        node.get_logger().info(
            f"경로 중심점: x={request.x:.3f}, y={request.y:.3f}, z={request.z:.3f}"
        )

        pending_tree_location["data"] = [request.x, request.y, request.z,
                        request.a, request.b, request.c]

        response.success = True
        response.message = "Tamping task completed."
        return response

    # 서비스 서버 생성 (이제 node는 우리가 만든 rclpy Node)
    srv = node.create_service(
        TreeLocation,
        "/tamping_task",      # 또는 f"/{ROBOT_ID}/tamping_task"
        handle_tamping_task
    )

    node.get_logger().info("tamping_task service ready.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if pending_tree_location["data"] is not None:
                tl = pending_tree_location["data"]
                pending_tree_location["data"] = None
                try:
                    perform_task(tl)
                    node.get_logger().info("Tamping task completed.")
                except Exception as e:
                    node.get_logger().error(f"perform_task error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()