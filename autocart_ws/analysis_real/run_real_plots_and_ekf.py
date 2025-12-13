#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os
import math

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(BASE_DIR, "csv")
PLOT_DIR = os.path.join(BASE_DIR, "plots_real")
os.makedirs(PLOT_DIR, exist_ok=True)

gps_path = os.path.join(CSV_DIR, "real_gps_all.csv")
imu_path = os.path.join(CSV_DIR, "real_imu_all.csv")

gps = np.genfromtxt(gps_path, delimiter=",", names=True)
t_gps_ns = gps["time_ns"]
lat = gps["lat"]
lon = gps["lon"]

t_gps = (t_gps_ns - t_gps_ns[0]) / 1e9

lat0 = lat[0] * math.pi / 180.0
lon0 = lon[0] * math.pi / 180.0
R = 6378137.0 

lat_rad = lat * math.pi / 180.0
lon_rad = lon * math.pi / 180.0

dlat = lat_rad - lat0
dlon = lon_rad - lon0

x_gps = R * dlon * math.cos(lat0)
y_gps = R * dlat

imu = np.genfromtxt(imu_path, delimiter=",", names=True)
t_imu_ns = imu["time_ns"]
ax = imu["ax"]
ay = imu["ay"]
az = imu["az"]
wx = imu["wx"]
wy = imu["wy"]
wz = imu["wz"]

t_imu = (t_imu_ns - t_imu_ns[0]) / 1e9

plt.figure(figsize=(8,6))
plt.scatter(x_gps, y_gps, s=8, alpha=0.6)
plt.title("Real GPS Trajectory (Local XY Frame)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.axis("equal")
plt.savefig(os.path.join(PLOT_DIR, "real_gps_scatter_xy.png"), dpi=300)

plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(t_imu, ax, linewidth=1.0)
plt.title("Real IMU Longitudinal Acceleration (ax)")
plt.ylabel("ax (m/s^2)")
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(t_imu, wz, linewidth=1.0, color="orange")
plt.title("Real IMU Yaw Rate (wz)")
plt.xlabel("Time (s)")
plt.ylabel("Yaw rate (rad/s)")
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(PLOT_DIR, "real_imu_ax_wz.png"), dpi=300)


N = len(t_gps)
x_f = np.zeros((N, 4)) 
P = np.eye(4) * 1.0

x_f[0,0] = x_gps[0]
x_f[0,1] = y_gps[0]
x_f[0,2] = 0.0
x_f[0,3] = 0.0

q_pos = 0.5
q_vel = 0.5
Q = np.diag([q_pos, q_pos, q_vel, q_vel])

r_pos = 0.5
Rk = np.diag([r_pos**2, r_pos**2])

innovations = []

for i in range(1, N):
    dt = t_gps[i] - t_gps[i-1]
    if dt <= 0 or dt > 1.0:
        dt = 0.1  

    x_prev = x_f[i-1]

    F = np.array([
        [1, 0, dt, 0 ],
        [0, 1, 0,  dt],
        [0, 0, 1,  0 ],
        [0, 0, 0,  1 ]
    ])

    x_pred = F @ x_prev
    P_pred = F @ P @ F.T + Q

    z = np.array([x_gps[i], y_gps[i]])
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z_pred = H @ x_pred
    y_res = z - z_pred
    S = H @ P_pred @ H.T + Rk
    K = P_pred @ H.T @ np.linalg.inv(S)

    x_new = x_pred + K @ y_res
    P = (np.eye(4) - K @ H) @ P_pred

    x_f[i] = x_new
    innovations.append(np.linalg.norm(y_res))

innovations = np.array(innovations)
x_ekf = x_f[:,0]
y_ekf = x_f[:,1]

plt.figure(figsize=(8,6))
plt.scatter(x_gps, y_gps, s=8, alpha=0.4, label="GPS (raw)")
plt.plot(x_ekf, y_ekf, linewidth=2.0, color="red", label="EKF smoothed")
plt.title("Real GPS vs EKF-Smoothed Trajectory")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.savefig(os.path.join(PLOT_DIR, "real_gps_vs_ekf_xy.png"), dpi=300)

t_mid = t_gps[1:] 

plt.figure(figsize=(10,4))
plt.plot(t_mid, innovations, linewidth=1.5)
plt.title("EKF Innovation Magnitude (Real Data)")
plt.xlabel("Time (s)")
plt.ylabel("Innovation norm (m)")
plt.grid(True)
plt.savefig(os.path.join(PLOT_DIR, "real_ekf_innovation.png"), dpi=300)

print("\nAll real-data plots saved in:", PLOT_DIR)

