#!/usr/bin/env python3
"""
Sakhir Racing Visualization
ROS 2 Humble / F1TENTH
Author: Byron Ortega

Visualizes:
- Vehicle trajectory, cleared after each lap
- Virtual start/finish line
- Floating speed and lap-time telemetry
- Automatic ten-lap counting

Does not publish driving commands.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class RaceVisualizer(Node):

    def __init__(self):
        super().__init__("sakhir_race_visualizer")

        self.publisher = self.create_publisher(
            MarkerArray, "/sakhir/race_markers", 10
        )

        self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.on_odometry,
            10
        )

        self.create_timer(0.25, self.publish_markers)

        self.origin = None
        self.position = None
        self.last_xy = None
        self.last_trace_xy = None
        self.frame_id = "map"

        self.trace = []
        self.distance = 0.0
        self.armed = False

        self.lap_start = None
        self.lap_times = []
        self.finished = False
        self.speed = 0.0

        print("\n" + "=" * 52)
        print("SAKHIR RACING TELEMETRY")
        print("Trajectory + Start/Finish + Lap Timer")
        print("Start this node BEFORE the racing controller.")
        print("Visualization topic: /sakhir/race_markers")
        print("=" * 52, flush=True)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    @staticmethod
    def point(x, y, z=0.15):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def reset(self, x, y, z, yaw):
        self.origin = (x, y, yaw)
        self.position = (x, y, z)
        self.last_xy = (x, y)
        self.last_trace_xy = (x, y)

        self.trace = [self.point(x, y)]
        self.distance = 0.0
        self.armed = False
        self.lap_start = None
        self.lap_times = []
        self.finished = False
        self.speed = 0.0

    def on_odometry(self, msg):
        if self.finished:
            return

        p = msg.pose.pose.position
        x, y, z = float(p.x), float(p.y), float(p.z)
        yaw = self.yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        self.frame_id = msg.header.frame_id or "map"

        if self.origin is None:
            self.reset(x, y, z, yaw)
            print("Start/finish line established.", flush=True)
            return

        xp, yp = self.last_xy
        step = math.hypot(x - xp, y - yp)

        # Detect simulation repositioning.
        if step > 3.0:
            self.reset(x, y, z, yaw)
            print("Position reset detected.", flush=True)
            return

        self.position = (x, y, z)

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self.speed = math.hypot(vx, vy)

        self.distance += step
        now = time.monotonic()

        if self.lap_start is None and step > 0.005:
            self.lap_start = now
            print("Race timer started.", flush=True)

        # Add a trajectory point every 15 cm.
        xt, yt = self.last_trace_xy
        if math.hypot(x - xt, y - yt) >= 0.15:
            self.trace.append(self.point(x, y))
            self.last_trace_xy = (x, y)

        x0, y0, yaw0 = self.origin
        c = math.cos(yaw0)
        s = math.sin(yaw0)

        before = (xp - x0) * c + (yp - y0) * s
        after = (x - x0) * c + (y - y0) * s

        lateral = -(x - x0) * s + (y - y0) * c

        heading_error = math.atan2(
            math.sin(yaw - yaw0),
            math.cos(yaw - yaw0)
        )

        if math.hypot(x - x0, y - y0) > 8.0:
            self.armed = True

        crossing = (
            self.armed
            and self.lap_start is not None
            and self.distance > 30.0
            and now - self.lap_start > 6.0
            and before < 0.0 <= after
            and abs(lateral) < 2.0
            and abs(heading_error) < 0.9
        )

        if crossing:
            lap_time = now - self.lap_start
            self.lap_times.append(lap_time)
            count = len(self.lap_times)

            print(
                f"LAP {count:02d}/10 | "
                f"Time: {lap_time:.2f} s | "
                f"Best: {min(self.lap_times):.2f} s",
                flush=True
            )

            # Clear the previous lap's visible route.
            self.trace = [self.point(x, y)]
            self.last_trace_xy = (x, y)

            self.distance = 0.0
            self.armed = False
            self.lap_start = now

            if count >= 10:
                total = sum(self.lap_times)
                print("\nTEN LAPS COMPLETED", flush=True)
                print(f"TOTAL: {total:.2f} s", flush=True)
                print(
                    f"AVERAGE: {total / 10:.2f} s",
                    flush=True
                )
                print(
                    f"BEST: {min(self.lap_times):.2f} s",
                    flush=True
                )
                self.finished = True

        self.last_xy = (x, y)

    def marker(self, marker_type, marker_id):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "sakhir_racing"
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.a = 1.0
        return m

    def publish_markers(self):
        if self.origin is None or self.position is None:
            return

        markers = MarkerArray()
        x, y, z = self.position
        x0, y0, yaw0 = self.origin

        # 1. Green trajectory that follows the car.
        trail = self.marker(Marker.LINE_STRIP, 1)
        trail.scale.x = 0.13
        trail.color.r = 0.02
        trail.color.g = 1.0
        trail.color.b = 0.43
        trail.points = self.trace
        markers.markers.append(trail)

        # 2. Checkered virtual start/finish line.
        perp_x = -math.sin(yaw0)
        perp_y = math.cos(yaw0)

        for i in range(8):
            a = -1.4 + 2.8 * i / 8
            b = -1.4 + 2.8 * (i + 1) / 8

            gate = self.marker(Marker.LINE_STRIP, 20 + i)
            gate.scale.x = 0.20

            if i % 2 == 0:
                gate.color.r = 1.0
                gate.color.g = 1.0
                gate.color.b = 1.0
            else:
                gate.color.r = 0.95
                gate.color.g = 0.15
                gate.color.b = 0.15

            gate.points = [
                self.point(
                    x0 + a * perp_x,
                    y0 + a * perp_y,
                    0.22
                ),
                self.point(
                    x0 + b * perp_x,
                    y0 + b * perp_y,
                    0.22
                )
            ]

            markers.markers.append(gate)

        # 3. Stationary START / FINISH sign.
        sign = self.marker(Marker.TEXT_VIEW_FACING, 50)
        sign.pose.position.x = x0 + 2.1 * perp_x
        sign.pose.position.y = y0 + 2.1 * perp_y
        sign.pose.position.z = 0.8
        sign.scale.z = 0.55
        sign.color.r = 1.0
        sign.color.g = 0.85
        sign.color.b = 0.10
        sign.text = "SAKHIR\nSTART / FINISH"
        markers.markers.append(sign)

        # 4. Floating telemetry attached to the car.
        label = self.marker(Marker.TEXT_VIEW_FACING, 51)
        label.pose.position.x = x + 0.55
        label.pose.position.y = y + 0.80
        label.pose.position.z = z + 1.0
        label.scale.z = 0.48
        label.color.r = 0.05
        label.color.g = 0.95
        label.color.b = 1.0

        count = len(self.lap_times)
        last = self.lap_times[-1] if count else None
        best = min(self.lap_times) if count else None

        if self.finished:
            total = sum(self.lap_times)
            label.text = (
                "SAKHIR - FINISHED 10/10\n"
                f"TOTAL: {total:.2f} s\n"
                f"AVERAGE: {total / 10:.2f} s\n"
                f"BEST: {best:.2f} s"
            )
        else:
            current = (
                time.monotonic() - self.lap_start
                if self.lap_start is not None
                else 0.0
            )

            last_text = (
                f"{last:.2f} s" if last is not None
                else "--"
            )
            best_text = (
                f"{best:.2f} s" if best is not None
                else "--"
            )

            label.text = (
                f"SAKHIR | LAP {min(count + 1, 10)}/10\n"
                f"SPEED: {self.speed:.1f} m/s "
                f"({self.speed * 3.6:.1f} km/h)\n"
                f"CURRENT: {current:.1f} s\n"
                f"LAST: {last_text} | BEST: {best_text}"
            )

        markers.markers.append(label)
        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RaceVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
