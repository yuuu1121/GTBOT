from setuptools import find_packages, setup

package_name = 'gtbot_formation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuuu1121',
    maintainer_email='yurnd25@gmail.com',
    description='Koopman formation control on Stonefish (leader-relative RLS+LP)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
