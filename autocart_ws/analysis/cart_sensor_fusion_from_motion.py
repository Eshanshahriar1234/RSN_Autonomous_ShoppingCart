#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# Same timing as motion script
dt = 0.1
T  = 12.0
N  = int(T / dt)
t  = np.linspace(0, T, N)

# ----- 1) True trajectory (must match motion script logic) -----
x_true = np.zeros(N)
y_true = np.zeros(N)

for k in range(N):
    tk = t[k]

    if tk <= 4.0:
        beta = tk / 4.0
        x_true[k] = -3.0 + beta * (4.0 + 3.0)   # -3 -> 4
        y_true[k] = -1.0
    elif tk <= 10.0:
        gamma = (tk - 4.0) / 6.0
        gamma = np.clip(gamma, 0.0, 1.0)
        x_true[k] = 4.0 + gamma * (10.0 - 4.0)  # 4 -> 10
        y_true[k] = -1.0 + gamma * (4.0 + 1.0)  # -1 -> 4
    else:
        x_true[k] = 10.0
        y_true[k] = 4.0

# True velocities and heading (for IMU reference)
vx_true = np.gradient(x_true, dt)
vy_true = np.gradient(y_true, dt)
speed_true = np.sqrt(vx_true**2 + vy_true**2)
heading_true = np.arctan2(vy_true, vx_true)
yaw_rate_true = np.gradient(heading_true, dt)

# ----- 2) Simulated GPS and IMU measurements -----
gps_sigma = 0.3      # position noise [m]
imu_v_sigma = 0.05   # speed noise [m/s]
imu_gyro_sigma = np.deg2rad(2.0)  # yaw-rate noise [rad/s]

gps_x = x_true + np.random.normal(0, gps_sigma, size=N)
gps_y = y_true + np.random.normal(0, gps_sigma, size=N)

imu_speed = speed_true + np.random.normal(0, imu_v_sigma, size=N)
imu_yaw_rate = yaw_rate_true + np.random.normal(0, imu_gyro_sigma, size=N)

# ----- 3) Simple 2D constant-velocity EKF -----
# State: [x, y, vx, vy]^T
F = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0,  1,  0],
              [0, 0,  0,  1]])

H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])   # GPS: position only

Q = np.diag([0.01, 0.01, 0.1, 0.1])        # process noise
R = np.diag([gps_sigma**2, gps_sigma**2])  # measurement noise

x_est = np.zeros((4, N))
P = np.eye(4)

# Initial guess from first GPS
x_est[:, 0] = [gps_x[0], gps_y[0], 0.0, 0.0]

for k in range(1, N):
    # --- Prediction ---
    # Convert noisy IMU speed + yaw_rate to vx, vy
    v = max(imu_speed[k], 0.0)
    # (For simplicity we still use true heading as a reference.
    #  You could also integrate imu_yaw_rate here.)
    theta = heading_true[k]
    vx_imu = v * np.cos(theta)
    vy_imu = v * np.sin(theta)

    x_pred = F @ x_est[:, k-1]
    # mix in IMU-based velocity (simple fusion in the process model)
    x_pred[2] = 0.7 * x_pred[2] + 0.3 * vx_imu
    x_pred[3] = 0.7 * x_pred[3] + 0.3 * vy_imu

    P_pred = F @ P @ F.T + Q

    # --- Update with GPS ---
    z = np.array([gps_x[k], gps_y[k]])
    y_res = z - H @ x_pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    x_est[:, k] = x_pred + K @ y_res
    P = (np.eye(4) - K @ H) @ P_pred

x_filt = x_est[0, :]
y_filt = x_est[1, :]

# ----- 4) PLOTS -----

# 4.1 GPS position noise over time
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(t, x_true, label='True X')
plt.scatter(t, gps_x, s=8, alpha=0.5, label='GPS X')
plt.ylabel('X [m]')
plt.legend()
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(t, y_true, label='True Y')
plt.scatter(t, gps_y, s=8, alpha=0.5, label='GPS Y')
plt.xlabel('Time [s]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('gps_position_noise.png', dpi=200)

# 4.2 IMU speed and yaw-rate vs true
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(t, speed_true, label='True speed')
plt.plot(t, imu_speed, linestyle='--', label='IMU speed (noisy)')
plt.ylabel('Speed [m/s]')
plt.legend()
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(t, yaw_rate_true, label='True yaw rate')
plt.plot(t, imu_yaw_rate, linestyle='--', label='IMU yaw rate (noisy)')
plt.xlabel('Time [s]')
plt.ylabel('Yaw rate [rad/s]')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('imu_measurements.png', dpi=200)

# 4.3 Trajectory plot: true vs GPS vs EKF
plt.figure(figsize=(8,6))
plt.plot(x_true, y_true, 'k-', label='True path')
plt.scatter(gps_x, gps_y, s=12, alpha=0.4, label='GPS (noisy)')
plt.plot(x_filt, y_filt, 'r-', linewidth=2, label='EKF fused path')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Autonomous Cart Trajectory: True vs GPS vs EKF')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.savefig('cart_trajectory_fusion.png', dpi=200)

# 4.4 EKF position error
pos_err = np.sqrt((x_filt - x_true)**2 + (y_filt - y_true)**2)
plt.figure(figsize=(8,4))
plt.plot(t, pos_err)
plt.xlabel('Time [s]')
plt.ylabel('Position error [m]')
plt.title('EKF Position Error over Time')
plt.grid(True)
plt.tight_layout()
plt.savefig('cart_ekf_error.png', dpi=200)

print("Saved plots:")
print("  gps_position_noise.png")
print("  imu_measurements.png")
print("  cart_trajectory_fusion.png")
print("  cart_ekf_error.png")

