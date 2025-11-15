import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetCurrentPosx

from dsr_gss.firebase_utils import get_firebase_db_reference


class RobotStatusUploader(Node):
    def __init__(self):
        super().__init__("robot_status_uploader")
        self.db_ref = get_firebase_db_reference()
        # 조인트 좌표계 구독 설정
        self.subscription = self.create_subscription(
            JointState, "/dsr01/joint_states", self.listener_callback, 10
        )
        self.subscription

        # 테스크 좌표계 업로드 설정
        # self.timer = self.create_timer(1.0, self.timer_callback)
        # self.task_coordinate_uploader = self.create_client(
        #     GetCurrentPosx, "task_coordinate_uploader"
        # )

    def listener_callback(self, msg):
        data = [pos for pos in msg.position]
        self.db_ref.child("joint_coordinate").set(data)

    def timer_callback(self):
        # self.get_logger().info(f"Calling Task Coordinate Uploader Service...")
        result = self.send_request()
        self.get_logger().info(f"Task Coordinate Uploader Response: {result}")

    def send_request(self):
        self.future = self.task_coordinate_uploader.call_async(GetCurrentPosx.Request())

        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            return self.future.result()
        else:
            self.get_logger().error(f"Service call failed: {self.future.exception()}")
            return None


def main(args=None):
    rclpy.init(args=args)

    node = RobotStatusUploader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
