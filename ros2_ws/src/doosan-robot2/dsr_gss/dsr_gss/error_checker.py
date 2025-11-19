import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import RobotError

import math
import time
from dsr_gss.firebase_utils import get_firebase_db_reference, info, error, exit


LEVEL_MAP = {1: "INFO", 2: "WARN", 3: "ERROR"}

GROUP_MAP = {1: "SYSTEM", 2: "MOTION", 3: "TP", 4: "INVERTER", 5: "CONTROLLER"}


class ErrorChecker(Node):
    def __init__(self):
        super().__init__("error_checker")
        self.subscription = self.create_subscription(
            RobotError, "/dsr01/error", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        # int32     level   # INFO =1, WARN =2, ERROR =3
        # int32     group   # SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5
        # int32     code    # error code
        # string    msg1    # error msg 1
        # string    msg2    # error msg 2
        # string    msg3    # error msg 3
        if msg.level == 1:
            return

        error(
            f"System error 발생!!! level: {msg.level}, group: {msg.group}, code: {msg.code}, msg1: '{msg.msg1}', msg2: '{msg.msg1}', msg3: '{msg.msg1}' "
        )
        exit()


def main(args=None):
    rclpy.init(args=args)

    node = ErrorChecker()

    try:
        info("ErrorChecker 시작합니다.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
