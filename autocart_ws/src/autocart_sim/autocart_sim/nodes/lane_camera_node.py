#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneCameraNode(Node):
    def __init__(self):
        super().__init__('lane_camera_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, 10)
        self.lane_offset_pub = self.create_publisher(Float32, '/perception/lane_offset', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/perception/obstacle_ahead', 10)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CV bridge error: {e}')
            return

        h, w, _ = cv_image.shape
        roi = cv_image[int(h*0.6):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        M = cv2.moments(mask)
        offset = 0.0
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            offset = (cx - w / 2.0) / (w / 2.0)

        lane_msg = Float32()
        lane_msg.data = float(offset)
        self.lane_offset_pub.publish(lane_msg)

        non_lane = cv2.bitwise_not(mask)
        non_lane_ratio = np.count_nonzero(non_lane) / non_lane.size
        obstacle = Bool()
        obstacle.data = non_lane_ratio > 0.8
        self.obstacle_pub.publish(obstacle)

def main(args=None):
    rclpy.init(args=args)
    node = LaneCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
