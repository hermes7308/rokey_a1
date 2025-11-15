# test_server.py

import rclpy
from rclpy.node import Node
from dsr_interfaces.srv import TreeLocation

class TestServer(Node):
    def __init__(self):
        super().__init__('test_server')

        # 서비스 서버 생성
        self.srv = self.create_service(
            TreeLocation,
            'tamping_task',
            self.callback
        )
        self.get_logger().info("Test service server started. Waiting for requests...")

    def callback(self, request, response):
        # 요청 들어온 값 출력
        self.get_logger().info(
            f"Received request: x={request.x}, y={request.y}, z={request.z}"
        )

        # 응답 생성
        response.success = True
        response.message = "Server received the coordinates successfully."

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TestServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
