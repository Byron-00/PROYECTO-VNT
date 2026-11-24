#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from gap_controller import SakhirGapController
from gap_controller_follow import SakhirFollowController


class BlueInitialPass(SakhirFollowController):
    def __init__(self):
        self.phase = "PASS"
        self.blue_xy = None
        self.orange_xy = None
        self.blue_yaw = None
        self.initial_heading = 1.49352478
        self.initial_xy = (0.70425148, 6.34556961)
        self.last_log = 0.0

        super().__init__()

        self.status_pub = self.create_publisher(
            Bool, "/sakhir/initial_pass_done", 10
        )
        self.create_subscription(
            Odometry, "/ego_racecar/odom",
            self.blue_callback, 10
        )
        self.create_subscription(
            Odometry, "/opp_racecar/odom",
            self.orange_callback, 10
        )

        self.get_logger().info(
            "BLUE: ONE STRAIGHT PASS, THEN V5 FOLLOW"
        )

    def blue_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.blue_xy = (p.x, p.y)
        self.blue_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0-2.0*(q.y*q.y + q.z*q.z)
        )

    def orange_callback(self, msg):
        p = msg.pose.pose.position
        self.orange_xy = (p.x, p.y)

    def advance(self):
        if self.blue_xy is None or self.orange_xy is None:
            return None

        c = math.cos(self.initial_heading)
        s = math.sin(self.initial_heading)

        bx, by = self.blue_xy
        ox, oy = self.orange_xy

        blue_progress = (
            (bx-self.initial_xy[0])*c
            + (by-self.initial_xy[1])*s
        )

        advantage = (bx-ox)*c + (by-oy)*s

        return blue_progress, advantage

    def publish_drive(self, speed, steering):
        if self.phase == "PASS":
            SakhirGapController.publish_drive(
                self,
                max(0.0, float(speed)),
                float(steering)
            )
        else:
            SakhirFollowController.publish_drive(
                self, speed, steering
            )

    def finish(self, success, reason):
        self.phase = "NORMAL"
        self.status_pub.publish(Bool(data=success))

        if success:
            self.get_logger().info(
                "OVERTAKE_COMPLETED - FOLLOW V5 FOR 10 LAPS"
            )
        else:
            self.get_logger().warn(
                "PASS_ABORT - FOLLOW V5: " + reason
            )

    def scan_callback(self, msg):
        if self.phase == "NORMAL":
            SakhirFollowController.scan_callback(self, msg)
            return

        state = self.advance()

        if state is None or self.blue_yaw is None:
            SakhirFollowController.scan_callback(self, msg)
            return

        progress, advantage = state

        # Solo un adelantamiento, confirmado por odometria.
        if advantage > 1.8:
            self.finish(True, "PASSED")
            SakhirFollowController.scan_callback(self, msg)
            return

        # No continuar la maniobra hacia la curva.
        if progress > 16.0:
            self.finish(False, "END OF INITIAL STRAIGHT")
            SakhirFollowController.scan_callback(self, msg)
            return

        heading_error = math.atan2(
            math.sin(self.initial_heading-self.blue_yaw),
            math.cos(self.initial_heading-self.blue_yaw)
        )

        if abs(heading_error) > 0.22:
            self.finish(False, "TRACK CURVE")
            SakhirFollowController.scan_callback(self, msg)
            return

        data = self.preprocess_scan(msg)
        if data is None:
            self.publish_drive(0.0, 0.0)
            return

        _, _, px, py = data

        # Mantener el carril paralelo de salida.
        steering = float(np.clip(
            0.9*heading_error, -0.10, 0.10
        ))

        clearance = self.trajectory_clearance(
            steering, px, py, self.current_speed()
        )

        # Si hay riesgo de obstaculo, recuperar V5.
        if not math.isfinite(clearance) or clearance < 2.0:
            self.finish(False, "PATH BLOCKED")
            SakhirFollowController.scan_callback(self, msg)
            return

        # Azul mas rapido que naranja mientras rebasa.
        speed = min(
            3.5,
            math.sqrt(2.0*4.0*max(clearance-1.0, 0.0))
        )

        self.publish_drive(speed, steering)

        now = time.monotonic()
        if now-self.last_log > 1.0:
            self.get_logger().info(
                f"INITIAL_PASS advantage={advantage:.2f} "
                f"progress={progress:.2f} "
                f"speed={speed:.2f} "
                f"clearance={clearance:.2f}"
            )
            self.last_log = now


def main():
    rclpy.init()
    node = BlueInitialPass()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
