#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import math
from collections import deque

class StuckDetectorNode(Node):
    def __init__(self):
        super().__init__('stuck_detector_node')
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom/fused', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.alert_pub = self.create_publisher(Bool, '/cart_stuck_alert', 10)
        self.last_cmd = Twist()
        self.pos_history = deque(maxlen=20)
        self.tilt_exceeded = False
        self.timer = self.create_timer(1.0, self.check_stuck)

    def cmd_cb(self,msg:Twist):
        self.last_cmd = msg

    def odom_cb(self,msg:Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pos_history.append((x,y))

    def imu_cb(self,msg:Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        g = math.sqrt(ax*ax + ay*ay + az*az)
        if abs(g - 9.8) > 3.0:
            self.tilt_exceeded = True

    def check_stuck(self):
        alert = Bool()
        alert.data = False
        move_cmd = abs(self.last_cmd.linear.x) > 0.1
        if len(self.pos_history) >= 2:
            x0,y0 = self.pos_history[0]
            x1,y1 = self.pos_history[-1]
            dist = math.hypot(x1-x0, y1-y0)
        else:
            dist = 0.0
        low_motion = dist < 0.2
        if move_cmd and low_motion:
            alert.data = True
        if self.tilt_exceeded:
            alert.data = True
        if alert.data:
            self.get_logger().warn('Cart appears STUCK – sending alert!')
            self.alert_pub.publish(alert)
            self.tilt_exceeded = False

def main(args=None):
    rclpy.init(args=args)
    node = StuckDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
