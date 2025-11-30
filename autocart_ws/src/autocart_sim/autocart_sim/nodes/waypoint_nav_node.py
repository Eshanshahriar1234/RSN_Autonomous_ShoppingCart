#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')
        self.waypoints = [(0.0,0.0),(5.0,0.0),(10.0,2.0),(12.0,5.0)]
        self.current_wp_idx = 0
        self.kp_lin = 0.8
        self.kp_ang = 1.2
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.obstacle_ahead = False
        self.create_subscription(Odometry, '/odom/fused', self.odom_cb, 10)
        self.create_subscription(Bool, '/perception/obstacle_ahead', self.obs_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_cb(self,msg:Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def obs_cb(self,msg:Bool):
        self.obstacle_ahead = msg.data

    def control_loop(self):
        cmd = Twist()
        if self.current_wp_idx >= len(self.waypoints):
            self.cmd_pub.publish(cmd)
            return
        if self.obstacle_ahead:
            self.cmd_pub.publish(cmd)
            return

        wx, wy = self.waypoints[self.current_wp_idx]
        dx = wx - self.pose_x
        dy = wy - self.pose_y
        dist = math.hypot(dx, dy)
        if dist < 0.5:
            self.current_wp_idx += 1
            self.cmd_pub.publish(cmd)
            return
        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self.yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
        lin_speed = self.kp_lin * dist
        ang_speed = self.kp_ang * yaw_err
        lin_speed = max(min(lin_speed, 0.8), -0.8)
        ang_speed = max(min(ang_speed, 1.5), -1.5)
        cmd.linear.x = float(lin_speed)
        cmd.angular.z = float(ang_speed)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
