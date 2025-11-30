from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Perception
        Node(package='autocart_sim', executable='lane_camera_node', name='lane_camera_node', output='screen'),

        # IMU-derived odom
        Node(package='autocart_sim', executable='imu_odom_node', name='imu_odom_node', output='screen'),

        # GPS wrapper
        Node(package='autocart_sim', executable='gps_odom_node', name='gps_odom_node', output='screen'),

        # EKF fusion
        Node(package='autocart_sim', executable='ekf_fusion_node', name='ekf_fusion_node', output='screen'),

        # Waypoint navigation
        Node(package='autocart_sim', executable='waypoint_nav_node', name='waypoint_nav_node', output='screen'),

        # Stuck detection and alert
        Node(package='autocart_sim', executable='stuck_detector_node', name='stuck_detector_node', output='screen'),
    ])
