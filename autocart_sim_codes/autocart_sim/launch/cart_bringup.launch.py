from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='autocart_sim', executable='lane_camera_node'),
        Node(package='autocart_sim', executable='imu_odom_node'),
        Node(package='autocart_sim', executable='gps_odom_node'),
        Node(package='autocart_sim', executable='ekf_fusion_node'),
        Node(package='autocart_sim', executable='waypoint_nav_node'),
        Node(package='autocart_sim', executable='stuck_detector_node'),
    ])
