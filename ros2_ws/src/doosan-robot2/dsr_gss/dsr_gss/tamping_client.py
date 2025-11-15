# tamping_client.py
import sys
import rclpy
from rclpy.node import Node
from dsr_interfaces.srv import TreeLocation
from math import pi

class TampingClient(Node):
    def __init__(self):
        super().__init__("tamping_client")
        self.cli = self.create_client(TreeLocation, "/tamping_task")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("오류 코드 1: 서비스 구동을 대기합니다.")

    def send_request(self, x, y, z, a, b, c):
        req = TreeLocation.Request()
        req.x = x
        req.y = y
        req.z = z
        req.a = a
        req.b = b
        req.c = c

        self.get_logger().info(f"Tamping 경로 중점 좌표: x:{x}, y:{y}, z:{z}")

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
def main(args=None):
    rclpy.init(args=args)
    node = TampingClient()

    # 예시 좌표 (원래는 비전/플래너에서 받은 값 넣으면 됨)
    x, y, z = 500.0, 200.0, 100.0
    a, b, c = 0.0, pi, 0.0

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    a = float(sys.argv[4])
    b = float(sys.argv[5])
    c = float(sys.argv[6])

    resp = node.send_request(x, y, z, a, b, c)
    node.get_logger().info(f"Result: success={resp.success}, message='{resp.message}'")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()