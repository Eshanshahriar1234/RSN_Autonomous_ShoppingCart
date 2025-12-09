from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('autocart_sim')

    # World file path
    world_path = os.path.join(pkg_share, 'worlds', 'stopandshop_brigham.world')

    # ros_gz_sim package share
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    # Launch Gazebo Harmonic with the world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r'],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        gazebo,
    ])
