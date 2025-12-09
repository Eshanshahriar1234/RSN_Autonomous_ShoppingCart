from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('autocart_sim')
    
    # World file path
    world_path = os.path.join(pkg_share, 'worlds', 'stopandshop_brigham.world')
    
    # SDF model path
    sdf_path = os.path.join(pkg_share, 'models', 'cart.sdf')
    
    # URDF file path (still needed for robot_state_publisher)
    urdf_path = os.path.join(pkg_share, 'models', 'cart.urdf')
    
    # Read URDF file
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    # Launch Gazebo with the world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r'],
        output='screen'
    )
    
    # Spawn the robot using SDF file
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_path,
            '-name', 'autocart',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])
