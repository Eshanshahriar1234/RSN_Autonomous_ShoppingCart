#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

R_EARTH = 6378137.0

class GpsOdomNode(Node):
    def __init__(self):
        super().__init__('gps_odom_node')
        self.declare_parameter('ref_lat', 42.3340)
        self.declare_parameter('ref_lon', -71.1040)
        self.ref_lat = math.radians(self.get_parameter('ref_lat').value)
        self.ref_lon = math.radians(self.get_parameter('ref_lon').value)
        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom/gps_odom', 10)

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        d_lat = lat - self.ref_lat
        d_lon = lon - self.ref_lon
        x = R_EARTH * d_lon * math.cos(self.ref_lat)
        y = R_EARTH * d_lat

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
