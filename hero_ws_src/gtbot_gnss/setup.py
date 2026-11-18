from glob import glob

from setuptools import find_packages, setup

package_name = 'gtbot_gnss'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['live_map.html']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.example')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='kwbnoa1234@gmail.com',
    description='ZED-F9P + 국토지리정보원 NTRIP RTK 측위',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_node = gtbot_gnss.gnss_node:main',
            'gnss_read = gtbot_gnss.gnss_read:main',
            'rtk_run = gtbot_gnss.rtk_run:main',
            'plot_track = gtbot_gnss.plot_track:main',
        ],
    },
)
