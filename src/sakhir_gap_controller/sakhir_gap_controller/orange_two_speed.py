#!/usr/bin/env python3
import time
import rclpy

from std_msgs.msg import Bool
from gap_controller import SakhirGapController


class OrangeTwoSpeed(SakhirGapController):
    def __init__(self):
        self.released = False
        self.start_time = time.monotonic()
        super().__init__()

        self.create_subscription(
            Bool, "/sakhir/initial_pass_done",
            self.on_pass, 10
        )

        self.get_logger().info(
            "ORANGE: 1.0 m/s UNTIL PASS, THEN 4.0"
        )

    def on_pass(self, msg):
        if not self.released:
            self.released = True
            self.get_logger().info(
                "ORANGE RELEASED - PASS RESULT: "
                + str(msg.data)
            )

    def publish_drive(self, speed, steering):
        # Liberacion si el rebase falla o no llega la senal.
        if time.monotonic()-self.start_time > 15.0:
            if not self.released:
                self.released = True
                self.get_logger().warn(
                    "PASS TIMEOUT - ORANGE RELEASED"
                )

        limit = 4.0 if self.released else 1.0

        super().publish_drive(
            min(max(0.0, float(speed)), limit),
            steering
        )


def main():
    rclpy.init()
    node = OrangeTwoSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
