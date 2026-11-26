from pathlib import Path
from contextlib import ExitStack
from collections import Counter
import csv
import math
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

if len(sys.argv) != 3:
    raise SystemExit(
        "Uso: python3 export_motion_csv.py CARPETA_ROSBAG CARPETA_SALIDA"
    )

bag = Path(sys.argv[1]).expanduser().resolve()
output = Path(sys.argv[2]).expanduser().resolve()
databases = sorted(bag.glob("*.db3"))

if not databases:
    raise SystemExit("No se encontraron archivos .db3 en esa carpeta.")

if output.exists():
    raise SystemExit(
        f"La carpeta de salida ya existe: {output}\n"
        "No se sobrescribieron archivos."
    )

types = {
    "nav_msgs/msg/Odometry": Odometry,
    "ackermann_msgs/msg/AckermannDriveStamped": AckermannDriveStamped,
}

common_columns = [
    "bag_time_ns",
    "elapsed_s",
    "header_time_ns",
]

odom_columns = [
    "x_m", "y_m", "z_m",
    "qx", "qy", "qz", "qw", "yaw_rad",
    "vx_mps", "vy_mps", "vz_mps", "speed_mps",
    "wx_radps", "wy_radps", "wz_radps",
]

drive_columns = [
    "steering_angle_rad",
    "steering_angle_velocity_radps",
    "commanded_speed_mps",
    "commanded_acceleration_mps2",
    "commanded_jerk_mps3",
]

# Obtener el inicio común de la grabación.
starts = []

for db in databases:
    connection = sqlite3.connect(
        db.as_uri() + "?mode=ro", uri=True
    )
    try:
        value = connection.execute(
            "SELECT MIN(timestamp) FROM messages"
        ).fetchone()[0]
        if value is not None:
            starts.append(value)
    finally:
        connection.close()

if not starts:
    raise SystemExit("La grabación no contiene mensajes.")

start = min(starts)
output.mkdir(parents=True)

pending = output / "EXPORT_INCOMPLETE.txt"
pending.write_text(
    "La exportacion no ha terminado.\n",
    encoding="utf-8",
)

counts = Counter()
writers = {}
total = 0

with ExitStack() as stack:
    for db in databases:
        print("Leyendo:", db.name, flush=True)

        connection = sqlite3.connect(
            db.as_uri() + "?mode=ro", uri=True
        )

        try:
            rows = connection.execute("""
                SELECT t.name, t.type, m.timestamp, m.data
                FROM messages AS m
                JOIN topics AS t ON t.id = m.topic_id
                WHERE t.type IN (
                    'nav_msgs/msg/Odometry',
                    'ackermann_msgs/msg/AckermannDriveStamped'
                )
                ORDER BY m.timestamp, m.id
            """)

            for topic, typename, timestamp, data in rows:
                msg = deserialize_message(
                    bytes(data), types[typename]
                )
                is_odom = typename == "nav_msgs/msg/Odometry"

                if topic not in writers:
                    filename = (
                        topic.strip("/").replace("/", "_") + ".csv"
                    )

                    handle = stack.enter_context(
                        (output / filename).open(
                            "w",
                            newline="",
                            encoding="utf-8",
                        )
                    )

                    writer = csv.writer(handle)
                    writer.writerow(
                        common_columns
                        + (odom_columns if is_odom else drive_columns)
                    )
                    writers[topic] = writer

                stamp = msg.header.stamp

                row = [
                    timestamp,
                    (timestamp - start) / 1_000_000_000,
                    stamp.sec * 1_000_000_000 + stamp.nanosec,
                ]

                if is_odom:
                    p = msg.pose.pose.position
                    q = msg.pose.pose.orientation
                    v = msg.twist.twist.linear
                    w = msg.twist.twist.angular

                    yaw = math.atan2(
                        2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z),
                    )

                    speed = math.sqrt(
                        v.x**2 + v.y**2 + v.z**2
                    )

                    row += [
                        p.x, p.y, p.z,
                        q.x, q.y, q.z, q.w, yaw,
                        v.x, v.y, v.z, speed,
                        w.x, w.y, w.z,
                    ]
                else:
                    drive = msg.drive

                    row += [
                        drive.steering_angle,
                        drive.steering_angle_velocity,
                        drive.speed,
                        drive.acceleration,
                        drive.jerk,
                    ]

                writers[topic].writerow(row)
                counts[topic] += 1
                total += 1

                if total % 100000 == 0:
                    print(
                        "Filas exportadas:", total,
                        flush=True,
                    )

        finally:
            connection.close()

if not counts:
    raise SystemExit(
        "No se encontraron mensajes de movimiento o control."
    )

pending.unlink()

print("\nEXPORTACION COMPLETA")
for topic, count in sorted(counts.items()):
    print(f"{topic}: {count} filas")
print("Carpeta de salida:", output)
