from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('autocart_sim')

    # World file path
    world_path = os.path.join(pkg_share, 'worlds', 'stopandshop_brigham.world')

    # ros_gz_sim package share
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    # Launch Gazebo Harmonic with the world
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_path}'
        }.items()
    )

    return LaunchDescription([
        gz_launch,
    ])

