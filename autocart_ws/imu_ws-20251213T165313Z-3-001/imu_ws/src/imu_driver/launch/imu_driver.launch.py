from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for VectorNav IMU'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate'
        ),
        
        Node(
            package='imu_driver',
            executable='driver',
            name='imu_driver',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'frame_id': 'imu_link'
            }],
            output='screen'
        )
    ])
