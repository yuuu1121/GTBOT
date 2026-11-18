from glob import glob

from setuptools import find_packages, setup

package_name = 'thruster_pwm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='kwbnoa1234@gmail.com',
    description='T200 + Basic ESC 4채널 PWM 스러스터 제어 (PCA9685)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pwm_node = thruster_pwm_control.pwm_node:main',
            'keyboard_node = thruster_pwm_control.keyboard_node:main',
            'cascade_controller_node = thruster_pwm_control.cascade_controller_node:main',
            'heading_teleop_node = thruster_pwm_control.heading_teleop_node:main',
        ],
    },
)
