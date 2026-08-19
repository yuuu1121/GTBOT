from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pkrc_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PKRC Team',
    maintainer_email='pkrc@example.com',
    description='PKRC Real Robot Controller - Wall Following and Depth Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_following = pkrc_controller.wall_following_real:main',
            'depth_controller = pkrc_controller.depth_controller_real:main',
            'depth_controller_sim = pkrc_controller.depth_controller_sim:main',
            'teleop_depth = pkrc_controller.teleop_depth:main',
            'ukfm_localization = pkrc_controller.ukfm_localization:main',
            'ukfm_data_logger = pkrc_controller.ukfm_data_logger:main',
            'ekf_localization_real = pkrc_controller.ekf_localization:main',
            'comparison_data_logger = pkrc_controller.comparison_data_logger:main',
            'aruco_detector_6dof = pkrc_controller.aruco_detector_6dof:main',
            'pkrc_world_position = pkrc_controller.pkrc_world_position:main',
            'platform_follower = pkrc_controller.platform_follower:main',
            'pkrc_mover = pkrc_controller.pkrc_mover:main',
        ],
    },
)
