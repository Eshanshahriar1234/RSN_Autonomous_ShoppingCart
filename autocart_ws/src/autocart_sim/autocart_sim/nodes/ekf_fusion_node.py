#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math

class EkfFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')
        self.x = np.zeros((3,1))
        self.P = np.eye(3)
        self.Q = np.diag([0.05, 0.05, 0.01])
        self.R_gps = np.diag([1.0, 1.0])
        self.R_imu = np.array([[0.02]])
        self.R_lane = np.array([[0.1]])
        self.last_time = None

        self.create_subscription(Odometry, '/odom/wheel', self.wheel_cb, 10)
        self.create_subscription(Odometry, '/odom/imu_odom', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odom/gps_odom', self.gps_cb, 10)
        self.create_subscription(Float32, '/perception/lane_offset', self.lane_cb, 10)

        self.pub = self.create_publisher(Odometry, '/odom/fused', 10)

    def wheel_cb(self, msg: Odometry):
        if self.last_time is None:
            self.last_time = msg.header.stamp
            return
        dt = (msg.header.stamp.sec - self.last_time.sec) +              (msg.header.stamp.nanosec - self.last_time.nanosec) * 1e-9
        if dt <= 0.0:
            return
        self.last_time = msg.header.stamp
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        theta = self.x[2,0]
        B = np.array([
            [math.cos(theta)*dt, 0.0],
            [math.sin(theta)*dt, 0.0],
            [0.0, dt]
        ])
        u = np.array([[vx],[wz]])
        self.x = self.x + B @ u
        self.P = self.P + self.Q
        self.publish_fused(msg.header.stamp)

    def imu_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        z = np.array([[yaw]])
        H = np.array([[0.0,0.0,1.0]])
        self.update(z,H,self.R_imu,msg.header.stamp)

    def gps_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = np.array([[x],[y]])
        H = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0]])
        self.update(z,H,self.R_gps,msg.header.stamp)

    def lane_cb(self, msg: Float32):
        lane_offset = msg.data
        theta_lane = -lane_offset * 0.3
        z = np.array([[theta_lane]])
        H = np.array([[0.0,0.0,1.0]])
        self.update(z,H,self.R_lane,None)

    def update(self,z,H,R,stamp):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P
        if stamp is not None:
            self.publish_fused(stamp)

    def publish_fused(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.x[0,0])
        odom.pose.pose.position.y = float(self.x[1,0])
        theta = self.x[2,0]
        qz = math.sin(theta/2.0)
        qw = math.cos(theta/2.0)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = EkfFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
