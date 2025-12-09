# Autonomous Shopping Cart Return System (Simulation)  
### Simulating Stop & Shop – Brigham Circle, Boston  
### ROS2 + Gazebo + Classical Perception + EKF Fusion

---

## Project Overview

This project simulates an **autonomous shopping cart** that returns itself from a customer's parked car back to the **Cart Home Zone** outside **Stop & Shop, Brigham Circle – Boston**.

The cart:
- Navigates through a realistic parking lot simulation  
- Stays inside marked lanes  
- Avoids parked vehicles & pedestrians  
- Uses multi-sensor fusion (camera + IMU + GPS + odom)  
- Returns automatically to the Cart-Home Zone  
- Detects if it becomes **stuck** and sends an alert  

The system is **fully simulation-based**.  
No hardware is required.

---

## Technologies Used
- **ROS2**
- **Gazebo/Ignition**
- **Python**
- **Classical Computer Vision (HSV thresholding + optical flow)**
- **Sensor Fusion (Extended Kalman Filter)**
- **PID Control**
- **Custom Gazebo World (Stop & Shop Brigham Circle)**

---

## Project Structure

```
autocart_sim/
 ├── launch/
 │    ├── gazebo_world.launch.py
 │    └── cart_bringup.launch.py
 ├── nodes/
 │    ├── lane_camera_node.py
 │    ├── imu_odom_node.py
 │    ├── gps_odom_node.py
 │    ├── ekf_fusion_node.py
 │    ├── waypoint_nav_node.py
 │    └── stuck_detector_node.py
 ├── models/
 │    └── cart.urdf
 ├── worlds/
 │    └── stopandshop_brigham.world
 ├── package.xml
 └── setup.py
```

---

## Custom World: Stop & Shop Brigham Circle

We manually created a simple Gazebo world inspired by:
- The **parking lane layout**
- The **storefront Cart Home area**
- **Static parked cars**
- Simple walk paths for pedestrians

This mirrors the real usability scenario.

---

## Installation

### 1. Clone Repository
```bash
cd ~/ros2_ws/src
git clone <your_repo_link>
```

### 2. Build
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running The Simulation

### 1. Launch Gazebo world + spawn cart + bringup nodes
```bash
ros2 launch autocart_sim gazebo_world.launch.py
```

This will:
- Open the Stop & Shop parking lot  
- Spawn the autonomous cart  
- Start all sensors + fusion + navigation nodes  

---

## Nodes Overview

### **1. lane_camera_node.py**
- Extracts lane markings (HSV threshold)
- Computes cart's lane offset
- Rough obstacle detection straight ahead

### **2. imu_odom_node.py**
- Integrates IMU gyro → yaw
- Publishes `/odom/imu_odom`

### **3. gps_odom_node.py**
- Converts GPS → map frame using Brigham Circle latitude/longitude reference  
- Publishes `/odom/gps_odom`

### **4. ekf_fusion_node.py**
Fuses:
- IMU yaw  
- GPS global position  
- Camera lane-heading  
- Wheel odometry  

Output: `/odom/fused`

### **5. waypoint_nav_node.py**
- Takes the fused pose  
- PID controller drives cart through waypoints → Cart Home Zone  

### **6. stuck_detector_node.py**
Detects:
- Cart blocked by cars  
- Cart wedged  
- Cart tilted on curb  
- Cart not moving while commanded  

Publishes: `/cart_stuck_alert`

---

## Recommended Demo Flow

1. Start at a parking spot (near car)  
2. Cart begins autonomous return  
3. Cart follows lanes using camera + EKF  
4. Car pulls out → cart stops  
5. Pedestrian crosses → cart stops  
6. Obstacle placed → cart navigates around it  
7. Simulate cart getting stuck → stuck alert triggered  
8. Final arrival at Cart Home zone  

---

## Evaluation Metrics

- RMSE of fused pose vs ground truth  
- Lane following accuracy  
- Obstacle detection correctness  
- Stuck detection accuracy  
- Completion time  

---

## Team Members
- RM – Simulation world, camera perception  
- Rohit – IMU processing  
- Dhanush – GPS + navigation  
- Ishaan – EKF fusion + safety  

---

## License
MIT License

