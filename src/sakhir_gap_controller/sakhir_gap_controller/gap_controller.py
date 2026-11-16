
#!/usr/bin/env python3

"""
Sakhir Racing Controller V5
Author: Byron Ortega
Project: PROYECTO-VNT
ROS 2 Humble - F1TENTH Gym

Reactive Follow the Gap with:
- Disparity Extender
- Stable gap selection
- Corridor centering
- Damped adaptive heading
- Ackermann trajectory checking
- Predictive corner speed control
"""

import math
import time

import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SakhirGapController(Node):

    def __init__(self):
        super().__init__("sakhir_gap_controller")

        defaults = {
            "max_speed": 11.0,
            "max_lateral_accel": 7.5,
            "brake_accel": 7.0,
            "drive_accel": 7.0,
            "max_steering": 0.418,
            "steering_rate": 3.2,
            "wheelbase": 0.3302,
            "vehicle_width": 0.31,
            "vehicle_length": 0.58,
            "lidar_offset": 0.275,
            "footprint_center": 0.16,
            "safety_margin": 0.07,
            "bubble_radius": 0.30,
            "disparity_threshold": 0.30,
            "gap_weight": 3.0,
            "planning_horizon": 12.0,
            "lookahead_base": 1.15,
            "lookahead_gain": 0.13
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def p(name):
            return float(self.get_parameter(name).value)

        self.max_speed = p("max_speed")
        self.lat_accel = p("max_lateral_accel")
        self.brake = p("brake_accel")
        self.accel = p("drive_accel")

        self.max_steering = p("max_steering")
        self.steering_rate = p("steering_rate")
        self.wheelbase = p("wheelbase")

        self.width = p("vehicle_width")
        self.length = p("vehicle_length")
        self.lidar_offset = p("lidar_offset")
        self.footprint_center = p("footprint_center")
        self.margin = p("safety_margin")

        self.bubble = p("bubble_radius")
        self.disparity = p("disparity_threshold")
        self.gap_weight = p("gap_weight")
        self.horizon = p("planning_horizon")

        self.lookahead_base = p("lookahead_base")
        self.lookahead_gain = p("lookahead_gain")

        self.last_steering = 0.0
        self.last_command_speed = 0.0
        self.last_heading = 0.0
        self.last_gap_angle = 0.0

        self.actual_speed = 0.0
        self.last_odom = 0.0
        self.last_scan = time.monotonic()
        self.last_control = None
        self.last_log = 0.0

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.odom_callback,
            10
        )

        self.create_timer(0.10, self.watchdog)

        self.get_logger().info(
            "Sakhir Racing Controller V5 initialized"
        )

    def odom_callback(self, msg):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)

        self.actual_speed = math.hypot(vx, vy)
        self.last_odom = time.monotonic()

    def current_speed(self):
        if time.monotonic() - self.last_odom < 0.4:
            return self.actual_speed

        return self.last_command_speed

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ego_racecar/base_link"

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.publisher.publish(msg)

    def stop(self, reason):
        self.last_command_speed = 0.0
        self.publish_drive(0.0, self.last_steering)

        self.log_status(
            reason,
            0.0,
            0.0,
            0.0
        )

    def watchdog(self):
        if time.monotonic() - self.last_scan > 0.5:
            self.stop("SCAN_TIMEOUT")

    def log_status(self, mode, speed, front, clearance):
        now = time.monotonic()

        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f"{mode} | cmd={speed:.2f} "
                f"| actual={self.current_speed():.2f} "
                f"| steer={self.last_steering:.3f} "
                f"| front={front:.2f} "
                f"| path={clearance:.2f}"
            )

            self.last_log = now

    def preprocess_scan(self, msg):
        raw = np.asarray(msg.ranges, dtype=float)

        if len(raw) < 20 or msg.angle_increment <= 0:
            return None

        angles = (
            msg.angle_min
            + np.arange(len(raw)) * msg.angle_increment
        )

        scan_cap = 18.0

        if math.isfinite(msg.range_max) and msg.range_max > 0:
            scan_cap = min(
                scan_cap,
                float(msg.range_max)
            )

        valid = (
            np.isfinite(raw)
            & (raw > max(float(msg.range_min), 0.05))
        )

        clean = np.where(
            valid,
            np.clip(raw, 0.0, scan_cap),
            0.0
        )

        clean = np.where(
            np.isposinf(raw),
            scan_cap,
            clean
        )

        # LiDAR points transformed to rear-axle frame.
        obstacles = (
            valid
            & (raw < scan_cap)
        )

        px = (
            self.lidar_offset
            + raw[obstacles] * np.cos(angles[obstacles])
        )

        py = (
            raw[obstacles] * np.sin(angles[obstacles])
        )

        relevant = (
            (px > -1.0)
            & (px < self.horizon + 2.0)
            & (np.abs(py) < 9.0)
        )

        px = px[relevant][::2]
        py = py[relevant][::2]

        view = (
            np.abs(angles) <= math.radians(105)
        )

        ranges = clean[view]
        scan_angles = angles[view]

        # Remove isolated distant readings.
        left = np.concatenate((
            ranges[:1], ranges[:-1]
        ))

        right = np.concatenate((
            ranges[1:], ranges[-1:]
        ))

        ranges = np.minimum.reduce((
            ranges,
            left,
            right
        ))

        return ranges, scan_angles, px, py

    def disparity_extension(self, ranges, angles, speed):
        safe = ranges.copy()

        step = abs(
            float(angles[1] - angles[0])
        )

        edges = np.where(
            np.abs(np.diff(ranges)) > self.disparity
        )[0]

        for i in edges:
            a = float(ranges[i])
            b = float(ranges[i + 1])

            if a <= 0.0 or b <= 0.0:
                continue

            close = min(a, b)

            extension_angle = math.atan2(
                self.bubble,
                max(close, 0.10)
            )

            count = int(math.ceil(
                extension_angle / step
            ))

            if a < b:
                start = i + 1
                end = min(
                    len(safe),
                    i + count + 2
                )
            else:
                start = max(0, i - count + 1)
                end = i + 1

            safe[start:end] = np.minimum(
                safe[start:end],
                close
            )

        # Nearest-obstacle safety bubble.
        valid = np.where(
            (ranges > 0.0)
            & (ranges < 8.0)
        )[0]

        if valid.size:
            nearest = valid[
                np.argmin(ranges[valid])
            ]

            distance = max(
                float(ranges[nearest]),
                0.10
            )

            angle_radius = math.atan2(
                self.bubble,
                distance
            )

            mask = (
                np.abs(
                    angles - angles[nearest]
                ) <= angle_radius
            )

            safe[mask] = 0.0

        threshold = float(np.clip(
            0.85 + 0.10 * speed,
            0.85,
            1.90
        ))

        safe[safe < threshold] = 0.0

        return safe

    def find_best_gap(self, safe, angles):
        available = safe > 0.0

        padded = np.concatenate((
            [False],
            available,
            [False]
        ))

        changes = np.diff(
            padded.astype(int)
        )

        starts = np.where(changes == 1)[0]
        ends = np.where(changes == -1)[0] - 1

        best_angle = None
        best_score = -float("inf")

        for start, end in zip(starts, ends):
            if end - start < 8:
                continue

            span = (
                float(angles[end])
                - float(angles[start])
            )

            if span < math.radians(10):
                continue

            margin = max(
                3,
                int(0.18 * (end - start))
            )

            lo = start + margin
            hi = end - margin + 1

            if lo >= hi:
                continue

            center = (lo + hi - 1) // 2

            target = float(angles[center])

            depth = float(np.percentile(
                safe[lo:hi],
                20
            ))

            continuity = abs(
                target - self.last_gap_angle
            )

            score = (
                2.0 * span
                + 0.22 * min(depth, 10.0)
                - 0.90 * abs(target)
                - 0.90 * continuity
            )

            if score > best_score:
                best_score = score
                best_angle = target

        return best_angle

    def corridor_heading(self, px, py, speed):
        """
        Estimate corridor center from nearby
        left and right LiDAR wall points.
        """

        probe = float(np.clip(
            1.0 + 0.14 * speed,
            1.0,
            2.6
        ))

        band = (
            np.abs(px - probe) < 0.40
        )

        left = py[
            band & (py > 0.30)
        ]

        right = py[
            band & (py < -0.30)
        ]

        if left.size < 2 or right.size < 2:
            return None

        left_wall = float(
            np.percentile(left, 10)
        )

        right_wall = float(
            np.percentile(right, 90)
        )

        width = left_wall - right_wall

        if not 0.8 < width < 10.0:
            return None

        center_y = (
            left_wall + right_wall
        ) / 2.0

        return math.atan2(center_y, probe)

    def frontal_clearance(self, px, py):
        corridor = (
            (px > 0.45)
            & (np.abs(py) < 0.42)
        )

        if not np.any(corridor):
            return self.horizon

        distance = float(
            np.min(px[corridor]) - 0.45
        )

        return max(0.0, distance)

    def target_to_steering(self, heading, speed):
        lookahead = (
            self.lookahead_base
            + self.lookahead_gain * speed
        )

        lookahead = float(np.clip(
            lookahead,
            1.0,
            2.60
        ))

        # React more strongly to tight turns.
        sharpness = min(
            abs(heading) / 0.75,
            1.0
        )

        lookahead *= (
            1.0 - 0.55 * sharpness
        )

        lookahead = float(np.clip(
            lookahead,
            0.70,
            2.60
        ))

        steering = math.atan2(
            2.0
            * self.wheelbase
            * math.sin(heading),
            lookahead
        )

        return float(np.clip(
            steering,
            -self.max_steering,
            self.max_steering
        ))

    def trajectory_clearance(self, target_steering, px, py, speed):
        """
        Approximate Ackermann vehicle trajectory
        and rectangular collision envelope.

        Positions are expressed relative to
        the rear axle, not the LiDAR.
        """

        if px.size == 0:
            return self.horizon

        ds = 0.25
        count = int(self.horizon / ds)

        rear_x = 0.0
        rear_y = 0.0
        yaw = 0.0

        steering = self.last_steering

        half_length = (
            self.length / 2.0
            + self.margin
        )

        half_width = (
            self.width / 2.0
            + self.margin
        )

        prediction_speed = max(speed, 2.0)

        for index in range(count + 1):
            distance = index * ds

            c = math.cos(yaw)
            s = math.sin(yaw)

            # Approximate car-body center.
            cx = (
                rear_x
                + self.footprint_center * c
            )

            cy = (
                rear_y
                + self.footprint_center * s
            )

            dx = px - cx
            dy = py - cy

            local_x = c * dx + s * dy
            local_y = -s * dx + c * dy

            collision = (
                (np.abs(local_x) <= half_length)
                & (np.abs(local_y) <= half_width)
            )

            if np.any(collision):
                return distance

            if index == count:
                break

            # Physical steering-rate approximation.
            max_change = (
                self.steering_rate
                * ds / prediction_speed
            )

            steering += float(np.clip(
                target_steering - steering,
                -max_change,
                max_change
            ))

            dtheta = (
                ds * math.tan(steering)
                / self.wheelbase
            )

            midpoint = yaw + 0.5 * dtheta

            rear_x += ds * math.cos(midpoint)
            rear_y += ds * math.sin(midpoint)
            yaw += dtheta

        return self.horizon

    def select_steering(self, desired, px, py, speed):
        offsets = np.linspace(
            -0.14,
            0.14,
            9
        )

        candidates = np.concatenate((
            desired + offsets,
            [desired, self.last_steering, 0.0]
        ))

        candidates = np.unique(np.clip(
            candidates,
            -self.max_steering,
            self.max_steering
        ))

        best_steering = None
        best_clearance = 0.0
        best_score = -float("inf")

        for steering in candidates:
            clearance = self.trajectory_clearance(
                float(steering),
                px,
                py,
                speed
            )

            if clearance < 0.75:
                continue

            score = (
                0.22 * min(clearance, 10.0)
                - 13.0 * abs(steering - desired)
                - 0.70 * abs(
                    steering - self.last_steering
                )
            )

            if score > best_score:
                best_score = score
                best_steering = float(steering)
                best_clearance = clearance

        return best_steering, best_clearance

    def smooth_steering(self, desired, front, dt):
        alpha = (
            0.75 if front < 4.0
            else 0.55
        )

        filtered = (
            alpha * desired
            + (1.0 - alpha) * self.last_steering
        )

        maximum_change = (
            self.steering_rate * dt
        )

        steering = float(np.clip(
            filtered,
            self.last_steering - maximum_change,
            self.last_steering + maximum_change
        ))

        return float(np.clip(
            steering,
            -self.max_steering,
            self.max_steering
        ))

    def plan_speed(
        self,
        steering,
        desired,
        clearance,
        front,
        dt
    ):
        curvature = max(
            abs(math.tan(steering)),
            abs(math.tan(desired))
        ) / self.wheelbase

        curve_limit = math.sqrt(
            self.lat_accel
            / max(curvature, 1e-5)
        )

        available_path = max(
            clearance - 0.75,
            0.0
        )

        path_limit = math.sqrt(
            2.0 * self.brake * available_path
        )

        available_front = max(
            front - 0.55,
            0.0
        )

        front_limit = math.sqrt(
            2.0 * min(self.brake, 3.5) * available_front
        )

        # Avoid unnecessary braking when the planned
        # curved path has sufficient clearance.
        if (
            abs(desired) > 0.15
            and clearance > 2.5
            and front > 0.65
        ):
            front_limit = max(
                front_limit,
                min(curve_limit, 2.7)
            )

        desired_speed = min(
            self.max_speed,
            curve_limit,
            path_limit,
            front_limit
        )

        if desired_speed > self.last_command_speed:
            speed = min(
                desired_speed,
                self.last_command_speed + self.accel * dt
            )
        else:
            # Request braking immediately.
            speed = desired_speed

        return float(max(0.0, speed))

    def scan_callback(self, msg):
        now = time.monotonic()
        self.last_scan = now

        if self.last_control is None:
            dt = 0.05
        else:
            dt = float(np.clip(
                now - self.last_control,
                0.01,
                0.15
            ))

        self.last_control = now

        data = self.preprocess_scan(msg)

        if data is None:
            self.stop("INVALID_SCAN")
            return

        ranges, angles, px, py = data

        speed = self.current_speed()

        safe = self.disparity_extension(
            ranges,
            angles,
            speed
        )

        gap_angle = self.find_best_gap(
            safe,
            angles
        )

        if gap_angle is None:
            self.stop("NO_SAFE_GAP")
            return

        front = self.frontal_clearance(
            px,
            py
        )

        if front < 0.25:
            self.stop("EMERGENCY_STOP")
            return

        center_angle = self.corridor_heading(
            px,
            py,
            speed
        )

        # Damped adaptive heading:
        # strong response near an obstacle,
        # smaller corrections in open straights.
        weight_base = (
            self.gap_weight + 0.20 * speed
        )

        weight = float(np.clip(
            weight_base / (weight_base + front),
            0.28,
            0.95
        ))

        heading = weight * gap_angle

        # Center the vehicle on open straight sections.
        if (
            center_angle is not None
            and front > 3.5
            and abs(gap_angle) < 0.45
        ):
            heading = (
                0.45 * heading
                + 0.55 * center_angle
            )

        # Faster correction near sharp corners.
        response = (
            0.82
            if front < 4.0 or abs(gap_angle) > 0.30
            else 0.42
        )

        heading = (
            response * heading
            + (1.0 - response) * self.last_heading
        )

        desired = self.target_to_steering(
            heading,
            speed
        )

        selected, _ = self.select_steering(
            desired,
            px,
            py,
            speed
        )

        if selected is None:
            self.stop("NO_SAFE_TRAJECTORY")
            return

        steering = self.smooth_steering(
            selected,
            front,
            dt
        )

        clearance = self.trajectory_clearance(
            steering,
            px,
            py,
            speed
        )

        if clearance < 0.75:
            self.stop("PATH_BLOCKED")
            return

        command_speed = self.plan_speed(
            steering,
            desired,
            clearance,
            front,
            dt
        )

        self.last_steering = steering
        self.last_command_speed = command_speed
        self.last_heading = heading
        self.last_gap_angle = gap_angle

        self.publish_drive(
            command_speed,
            steering
        )

        self.log_status(
            "RACING_V5",
            command_speed,
            front,
            clearance
        )


def main(args=None):
    rclpy.init(args=args)

    node = SakhirGapController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
