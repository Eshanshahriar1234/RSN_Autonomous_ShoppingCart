import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# -----------------------
# CONFIGURATION
# -----------------------
dt = 0.02           # 50 Hz IMU, stable
T  = 12.0           # 12-second demo
steps = int(T / dt)

# GPS noise (very low realistic)
GPS_SIGMA = 0.08

# IMU noise (very low)
ACC_SIGMA = 0.02
YAW_SIGMA = 0.005

# Store data
true_x = []
true_y = []
gps_x = []
gps_y = []
imu_v = []
imu_w = []
t_all  = []

# Smooth trajectory: S-curve into store
for i in range(steps):
    t = i * dt
    t_all.append(t)

    # True motion path
    x = 0.3 * t**2
    y = -5 + 4 * np.sin(0.18 * t)

    true_x.append(x)
    true_y.append(y)

    # GPS (low noise)
    gps_x.append(x + np.random.normal(0, GPS_SIGMA))
    gps_y.append(y + np.random.normal(0, GPS_SIGMA))

    # IMU (derivatives, low noise)
    v_true = 0.6 * t
    yaw_true = 0.18 * np.cos(0.18 * t)

    imu_v.append(v_true + np.random.normal(0, ACC_SIGMA))
    imu_w.append(yaw_true + np.random.normal(0, YAW_SIGMA))

# Save CSV logs
pd.DataFrame({
    "time": t_all,
    "x": true_x,
    "y": true_y
}).to_csv("analysis/state_true.csv", index=False)

pd.DataFrame({
    "time": t_all,
    "x": gps_x,
    "y": gps_y
}).to_csv("analysis/gps.csv", index=False)

pd.DataFrame({
    "time": t_all,
    "v": imu_v,
    "w": imu_w
}).to_csv("analysis/imu.csv", index=False)

print("Simulated sensor data saved.")

