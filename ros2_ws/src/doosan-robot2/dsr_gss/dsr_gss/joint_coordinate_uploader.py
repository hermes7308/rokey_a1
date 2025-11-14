import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import time
from dsr_gss.firebase_utils import get_firebase_db_reference


class JointCoordinateUploader(Node):
    def __init__(self):
        super().__init__("joint_coordinate_uploader")
        self.db_ref = get_firebase_db_reference("robot_status")
        # 조인트 좌표계 구독 설정
        self.sub = self.create_subscription(
            JointState, "/dsr01/joint_states", self.listener_callback, 10
        )
        self.sub

    def listener_callback(self, msg):
        data = [pos for pos in msg.position]
        self.db_ref.child("dsr_gss").child("joint_coordinate").set({
            "data": data,
            "timestamp": int(time.time() * 1000)
        })


def main(args=None):
    rclpy.init(args=args)

    node = JointCoordinateUploader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
