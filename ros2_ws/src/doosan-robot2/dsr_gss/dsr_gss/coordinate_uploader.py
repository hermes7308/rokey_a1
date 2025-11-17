import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPosx
from dsr_msgs2.srv import GetCurrentPosj
from sensor_msgs.msg import JointState

import math
import time
from dsr_gss.firebase_utils import get_firebase_db_reference, info


class CoordinateUploader(Node):
    def __init__(self):
        super().__init__("coordinate_uploader")
        self.db_ref = get_firebase_db_reference()
        self.create_subscription(
            JointState, "/dsr01/joint_states", self.joint_callback, 10  # 절대 경로
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.task_cli = self.create_client(
            GetCurrentPosx, "dsr01/aux_control/get_current_posx"
        )

        self.task_req = GetCurrentPosx.Request()

    def joint_callback(self, msg):
        """joint_states 콜백: 데이터 저장만 수행"""
        # msg.name: ['joint_1', 'joint_2', 'joint_4', 'joint_5', 'joint_3', 'joint_6']
        # 0, 1, 4, 2, 3, 5
        global joint_cooridnates

        joint_index_list = [0, 1, 4, 2, 3, 5]
        joint_cooridnates = [
            math.degrees(msg.position[idx]) for idx in joint_index_list
        ]
        self.db_ref.child("joint_coordinate").set(
            {"data": joint_cooridnates, "timestamp": int(time.time() * 1000)}
        )

    def timer_callback(self):
        print("좌표계 업로더 동작 중...")

        while not self.task_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 대기 중: dsr01/aux_control/get_current_posx")
        task_future = self.task_cli.call_async(self.task_req)
        self.get_logger().info("서비스 호출 중: dsr01/aux_control/get_current_posx")
        task_future.add_done_callback(self.task_response_callback)

    def task_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                data = [p for p in response.task_pos_info[0].data]
                self.db_ref.child("task_coordinate").set(
                    {"data": data, "timestamp": int(time.time() * 1000)}
                )
                self.get_logger().info(f"Task 좌표계 데이터 수신(콜백): {data}")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 중 오류 발생(콜백): {e}")


def main(args=None):
    rclpy.init(args=args)

    node = CoordinateUploader()

    try:
        info("CoordinateUploader 시작합니다.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
