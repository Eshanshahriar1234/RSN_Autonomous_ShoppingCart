#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class ImuOdomNode(Node):
    def __init__(self):
        super().__init__('imu_odom_node')
        self.yaw = 0.0
        self.last_time = None
        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom/imu_odom', 10)

    def imu_callback(self, msg: Imu):
        if self.last_time is None:
            self.last_time = msg.header.stamp
            return
        dt = (msg.header.stamp.sec - self.last_time.sec) +              (msg.header.stamp.nanosec - self.last_time.nanosec) * 1e-9
        self.last_time = msg.header.stamp
        yaw_rate = msg.angular_velocity.z
        self.yaw += yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        qz = math.sin(self.yaw/2.0)
        qw = math.cos(self.yaw/2.0)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=float(qz), w=float(qw))
        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
