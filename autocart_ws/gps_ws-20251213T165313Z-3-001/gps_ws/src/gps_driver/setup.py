from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='GPS driver for EECE5554',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = gps_driver.driver:main',
        ],
    },
)
