import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'thruster_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='kwbnoa1234@gmail.com',
    description='8채널 ROV 스러스터 제어 — 키보드 텔레옵으로 RPM 명령을 발행하고 SocketCAN으로 전달',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'keyboard_node = thruster_control.keyboard_node:main',
            'thruster_can_node = thruster_control.thruster_can_node:main',
            'thruster_bridge = thruster_control.thruster_bridge_node:main',
        ],
    },
)
