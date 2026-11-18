from glob import glob

from setuptools import find_packages, setup

package_name = 'serial_cmd'

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
    description='CP2102 시리얼 포트로 텍스트 명령 전송 (여러 Pi 시각 지정 동시 실행 지원)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_cmd_node = serial_cmd.serial_cmd_node:main',
            'input_node = serial_cmd.input_node:main',
            'broadcast_node = serial_cmd.broadcast_node:main',
        ],
    },
)
