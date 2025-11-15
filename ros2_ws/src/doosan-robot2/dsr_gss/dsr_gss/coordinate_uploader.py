import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPosx
from dsr_msgs2.srv import GetCurrentPosj

import time
from dsr_gss.firebase_utils import get_firebase_db_reference


class CoordinateUploader(Node):
    def __init__(self):
        super().__init__("coordinate_uploader")
        self.db_ref = get_firebase_db_reference()
        self.timer = self.create_timer(3.0, self.timer_callback)

        self.task_cli = self.create_client(
            GetCurrentPosx, "dsr01/aux_control/get_current_posx"
        )
        self.joint_cli = self.create_client(
            GetCurrentPosj, "/dsr01/aux_control/get_current_posj"
        )

        self.task_req = GetCurrentPosx.Request()
        self.joint_req = GetCurrentPosj.Request()

        while not self.task_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 대기 중: dsr01/aux_control/get_current_posx")

        while not self.joint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 대기 중: dsr01/aux_control/get_current_posj")

    def timer_callback(self):
        print("좌표계 업로더 동작 중...")

        task_future = self.task_cli.call_async(self.task_req)
        print("서비스 호출 중: dsr01/aux_control/get_current_posx")
        task_future.add_done_callback(self.task_response_callback)

        joint_future = self.joint_cli.call_async(self.joint_req)
        print("서비스 호출 중: dsr01/aux_control/get_current_posj")
        joint_future.add_done_callback(self.joint_response_callback)

    def task_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                data = [p for p in response.task_pos_info[0].data]
                self.db_ref.child("task_coordinate").set(
                    {"data": data, "timestamp": int(time.time() * 1000)}
                )
                print(f"Task 좌표계 데이터 수신(콜백): {data}")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 중 오류 발생(콜백): {e}")

    def joint_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                data = [p for p in response.pos]
                self.db_ref.child("joint_coordinate").set(
                    {"data": data, "timestamp": int(time.time() * 1000)}
                )
                print(f"Joint 좌표계 데이터 수신(콜백): {data}")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 중 오류 발생(콜백): {e}")


def main(args=None):
    rclpy.init(args=args)

    node = CoordinateUploader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
