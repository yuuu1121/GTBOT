from setuptools import find_packages, setup

package_name = 'gtbot_formation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/formation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuuu1121',
    maintainer_email='yurnd25@gmail.com',
    description='Koopman formation control on Stonefish (leader-relative RLS+LP)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'velocity_loop = gtbot_formation.velocity_loop:main',
        'gate_s1 = gtbot_formation.gate_s1:main',
        'leader_pilot = gtbot_formation.leader_pilot:main',
        'koopman_formation = gtbot_formation.koopman_node:main',
        'gate_s3 = gtbot_formation.gate_s3:main',
        'platform_perception = gtbot_formation.platform_perception:main',
        'gate_s4 = gtbot_formation.gate_s4:main',
        'gate_s5 = gtbot_formation.gate_s5:main',
        'gate_s6 = gtbot_formation.gate_s6:main',
        'settle_wait = gtbot_formation.settle_wait:main',
    ]},
)
