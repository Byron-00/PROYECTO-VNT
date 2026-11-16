#!/usr/bin/env python3

"""
Sakhir automatic lap counter and lap timer.
Part 1 - F1TENTH Autonomous Racing.
Author: Byron Ortega

A virtual finish line is defined by the initial
vehicle position and heading. The car must start
from a straight section before the controller runs.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SakhirLapTimer(Node):

    def __init__(self):
        super().__init__("sakhir_lap_timer")

        self.start_pose = None
        self.last_position = None
        self.start_time = None
        self.distance = 0.0
        self.armed = False
        self.lap_times = []
        self.finished = False
        self.last_report = 0.0

        self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.odom_callback,
            10
        )

        self.create_timer(1.0, self.show_progress)

        print("\n" + "=" * 48)
        print("SAKHIR - AUTOMATIC LAP COUNTER")
        print("Target: 10 consecutive laps")
        print("Place the car at the starting line.")
        print("Start the controller after this node.")
        print("=" * 48, flush=True)

    @staticmethod
    def get_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def reset_counter(self, x, y, yaw):
        self.start_pose = (x, y, yaw)
        self.last_position = (x, y)
        self.start_time = None
        self.distance = 0.0
        self.armed = False
        self.lap_times.clear()
        self.finished = False

    def show_progress(self):
        if self.start_time is None or self.finished:
            return

        now = time.monotonic()
        if now - self.last_report >= 3.0:
            elapsed = now - self.start_time
            print(
                f"RUNNING | Laps: {len(self.lap_times)}/10 "
                f"| Current lap: {elapsed:.1f} s",
                flush=True
            )
            self.last_report = now

    def odom_callback(self, msg):
        if self.finished:
            return

        p = msg.pose.pose.position
        yaw = self.get_yaw(msg.pose.pose.orientation)

        x = float(p.x)
        y = float(p.y)

        if self.start_pose is None:
            self.reset_counter(x, y, yaw)
            print("Starting position recorded.", flush=True)
            return

        xp, yp = self.last_position
        step = math.hypot(x - xp, y - yp)

        # A large position jump indicates repositioning.
        if step > 3.0:
            self.reset_counter(x, y, yaw)
            print("Vehicle repositioned. Counter reset.", flush=True)
            return

        x0, y0, yaw0 = self.start_pose

        def along_track(xx, yy):
            return (
                (xx - x0) * math.cos(yaw0)
                + (yy - y0) * math.sin(yaw0)
            )

        before = along_track(xp, yp)
        after = along_track(x, y)

        lateral = (
            -(x - x0) * math.sin(yaw0)
            + (y - y0) * math.cos(yaw0)
        )

        heading_error = math.atan2(
            math.sin(yaw - yaw0),
            math.cos(yaw - yaw0)
        )

        self.distance += step
        now = time.monotonic()

        if self.start_time is None and step > 0.005:
            self.start_time = now
            print("\nTIMER STARTED\n", flush=True)

        # Arm the finish-line detector only after
        # moving away from the start position.
        if math.hypot(x - x0, y - y0) > 8.0:
            self.armed = True

        crossing = (
            self.armed
            and self.start_time is not None
            and self.distance > 30.0
            and now - self.start_time > 6.0
            and before < 0.0 <= after
            and abs(lateral) < 2.0
            and abs(heading_error) < 0.9
        )

        if crossing:
            lap_time = now - self.start_time
            self.lap_times.append(lap_time)
            count = len(self.lap_times)

            print(
                f"LAP {count:02d}/10 | "
                f"Time: {lap_time:.2f} s | "
                f"Best: {min(self.lap_times):.2f} s",
                flush=True
            )

            self.start_time = now
            self.distance = 0.0
            self.armed = False

            if count >= 10:
                total = sum(self.lap_times)

                print("\n" + "=" * 48)
                print("TEN LAPS COMPLETED")
                print(f"Total time: {total:.2f} s")
                print(f"Average lap: {total / 10:.2f} s")
                print(f"Best lap: {min(self.lap_times):.2f} s")
                print("=" * 48, flush=True)

                self.finished = True

        self.last_position = (x, y)


def main(args=None):
    rclpy.init(args=args)
    node = SakhirLapTimer()

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
