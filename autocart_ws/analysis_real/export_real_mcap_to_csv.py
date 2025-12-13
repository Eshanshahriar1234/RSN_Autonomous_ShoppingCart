import os
import csv
from mcap_ros2.reader import read_ros2_messages

gps_files = [
    "/home/pinky/autocart_ws/gps_ws-20251213T165313Z-3-001/gps_ws/bag1gps/bag1gps_0.mcap",
    "/home/pinky/autocart_ws/gps_ws-20251213T165313Z-3-001/gps_ws/bag2gps/bag2gps_0.mcap",
    "/home/pinky/autocart_ws/gps_ws-20251213T165313Z-3-001/gps_ws/bag3gps/bag3gps_0.mcap",
]

imu_files = [
    "/home/pinky/autocart_ws/imu_ws-20251213T165313Z-3-001/imu_ws/bag1imu/bag1imu_0.mcap",
    "/home/pinky/autocart_ws/imu_ws-20251213T165313Z-3-001/imu_ws/bag2imu/bag2imu_0.mcap",
    "/home/pinky/autocart_ws/imu_ws-20251213T165313Z-3-001/imu_ws/bag3imu/bag3imu_0.mcap",
]

os.makedirs("csv", exist_ok=True)

def extract_gps_from_mcap(path, writer):
    if not os.path.exists(path):
        print(f"[GPS] File not found, skipping: {path}")
        return

    print(f"[GPS] Reading {path}")
    count = 0
    for m in read_ros2_messages(path):
        try:
            msg = m.ros_msg
        except AttributeError:
            continue

        # We assume GPS bag contains NavSatFix-like messages
        # Guard against wrong message types
        if not hasattr(msg, "latitude") or not hasattr(msg, "longitude"):
            continue

        # Timestamp
        t_ns = getattr(m, "log_time_ns", None)
        if t_ns is None:
            # Fallback: fake timestamp using counter
            t_ns = count
            count += 1

        writer.writerow([t_ns, msg.latitude, msg.longitude, getattr(msg, "altitude", 0.0)])

def extract_imu_from_mcap(path, writer):
    if not os.path.exists(path):
        print(f"[IMU] File not found, skipping: {path}")
        return

    print(f"[IMU] Reading {path}")
    count = 0
    for m in read_ros2_messages(path):
        try:
            msg = m.ros_msg
        except AttributeError:
            continue

        # We assume IMU bag contains sensor_msgs/Imu-like messages
        if not hasattr(msg, "linear_acceleration") or not hasattr(msg, "angular_velocity"):
            continue

        t_ns = getattr(m, "log_time_ns", None)
        if t_ns is None:
            t_ns = count
            count += 1

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        writer.writerow([t_ns, ax, ay, az, wx, wy, wz])

gps_csv_path = "csv/real_gps_all.csv"
with open(gps_csv_path, "w", newline="") as f:
    gps_writer = csv.writer(f)
    gps_writer.writerow(["time_ns", "lat", "lon", "alt"])
    for p in gps_files:
        extract_gps_from_mcap(p, gps_writer)

print(f"[GPS] CSV written to {gps_csv_path}")

imu_csv_path = "csv/real_imu_all.csv"
with open(imu_csv_path, "w", newline="") as f:
    imu_writer = csv.writer(f)
    imu_writer.writerow(["time_ns", "ax", "ay", "az", "wx", "wy", "wz"])
    for p in imu_files:
        extract_imu_from_mcap(p, imu_writer)

print(f"[IMU] CSV written to {imu_csv_path}")

print("DONE.")

