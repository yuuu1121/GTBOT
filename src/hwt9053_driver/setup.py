import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hwt9053_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yju',
    maintainer_email='yju@todo.todo',
    description='HWT9053-485 AHRS IMU 센서 ROS2 드라이버 (sensor_msgs/Imu 퍼블리시)',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hwt9053_node = hwt9053_driver.hwt9053_node:main',
            'calibrate = hwt9053_driver.calibrate:main',
        ],
    },
)
