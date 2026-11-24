#!/usr/bin/env python3

import math
import time

import rclpy
from nav_msgs.msg import Odometry

from gap_controller import SakhirGapController


class SakhirFollowController(SakhirGapController):

    def __init__(self):
        super().__init__()

        self.ego_pose = None
        self.opp_pose = None

        self.ego_pose_time = 0.0
        self.opp_pose_time = 0.0

        self.opp_speed = 0.0

        self.follow_distance = 2.5
        self.emergency_distance = 1.0
        self.detection_distance = 12.0

        self.follow_log_time = 0.0

        self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.ego_pose_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/opp_racecar/odom",
            self.opponent_callback,
            10
        )

        self.get_logger().info(
            "V5 + FOLLOW DISTANCE READY"
        )

    @staticmethod
    def pose_from_odom(msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

        return float(p.x), float(p.y), yaw

    def ego_pose_callback(self, msg):
        self.ego_pose = self.pose_from_odom(msg)
        self.ego_pose_time = time.monotonic()

    def opponent_callback(self, msg):

        self.opp_pose = self.pose_from_odom(msg)
        self.opp_pose_time = time.monotonic()

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)

        self.opp_speed = math.hypot(vx, vy)

    def opponent_position(self):

        now = time.monotonic()

        if self.ego_pose is None or self.opp_pose is None:
            return None

        if now - self.ego_pose_time > 0.5:
            return None

        if now - self.opp_pose_time > 0.5:
            return None

        ex, ey, yaw = self.ego_pose
        ox, oy, opp_yaw = self.opp_pose

        dx = ox - ex
        dy = oy - ey

        forward = (
            math.cos(yaw)*dx
            + math.sin(yaw)*dy
        )

        lateral = (
            -math.sin(yaw)*dx
            + math.cos(yaw)*dy
        )

        heading_error = math.atan2(
            math.sin(opp_yaw-yaw),
            math.cos(opp_yaw-yaw)
        )

        return forward, lateral, heading_error

    def publish_drive(self, speed, steering):

        requested_speed = max(0.0, float(speed))
        allowed_speed = requested_speed

        # No avanzar sin odometria actualizada.
        other = self.opponent_position()

        if other is None:
            allowed_speed = 0.0

        else:
            forward, lateral, heading_error = other

            same_corridor = (
                0.0 < forward < self.detection_distance
                and abs(lateral) < 0.90
                and abs(heading_error) < 0.60
            )

            if same_corridor:

                # Distancia disponible entre carrocerias.
                gap = max(forward - 0.58, 0.0)

                # Limite de velocidad por seguimiento.
                error = gap - self.follow_distance

                follow_speed = max(
                    0.0,
                    self.opp_speed + 0.65*error
                )

                # Limite adicional de frenado.
                brake_speed = math.sqrt(
                    2.0 * min(self.brake, 4.0)
                    * max(gap - self.emergency_distance, 0.0)
                )

                allowed_speed = min(
                    requested_speed,
                    follow_speed,
                    brake_speed
                )

                if gap <= self.emergency_distance:
                    allowed_speed = 0.0

                now = time.monotonic()

                if now - self.follow_log_time >= 1.0:

                    self.get_logger().info(
                        f"FOLLOW_SAFE "
                        f"ahead={forward:.2f} "
                        f"gap={gap:.2f} "
                        f"opp_speed={self.opp_speed:.2f} "
                        f"cmd={allowed_speed:.2f}"
                    )

                    self.follow_log_time = now

        # Mantener coherente el ultimo comando del V5.
        self.last_command_speed = allowed_speed

        super().publish_drive(
            allowed_speed,
            steering
        )


def main(args=None):

    rclpy.init(args=args)

    node = SakhirFollowController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            try:
                node.publish_drive(0.0, 0.0)
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
