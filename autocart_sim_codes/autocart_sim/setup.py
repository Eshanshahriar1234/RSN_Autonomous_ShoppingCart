from setuptools import setup
import os
from glob import glob

package_name = 'autocart_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RM Hoque',
    maintainer_email='rmhoque@example.com',
    description='Autonomous shopping cart simulation package for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_camera_node = autocart_sim.nodes.lane_camera_node:main',
            'imu_odom_node = autocart_sim.nodes.imu_odom_node:main',
            'gps_odom_node = autocart_sim.nodes.gps_odom_node:main',
            'ekf_fusion_node = autocart_sim.nodes.ekf_fusion_node:main',
            'waypoint_nav_node = autocart_sim.nodes.waypoint_nav_node:main',
            'stuck_detector_node = autocart_sim.nodes.stuck_detector_node:main',
        ],
    },
)
