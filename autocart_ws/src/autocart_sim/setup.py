from setuptools import setup

package_name = 'autocart_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'autocart_sim/launch/gazebo_world.launch.py',
            'autocart_sim/launch/cart_bringup.launch.py',
        ]),
        ('share/' + package_name + '/worlds', [
            'autocart_sim/worlds/stopandshop_brigham.world',
        ]),
        ('share/' + package_name + '/models', [
            'autocart_sim/models/cart.urdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pinky',
    maintainer_email='aradhya.ro@northeastern.edu',
    description='Autonomous shopping cart return simulation',
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

