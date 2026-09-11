from glob import glob
from setuptools import setup

package_name = 'agrobot_husky_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Artur Martini da Rosa',
    maintainer_email='arturmartinidarosa@gmail.com',
    description='Lightweight GPS waypoint following for the Husky A300.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_waypoint_follower = agrobot_husky_nav.gps_waypoint_follower:main',
            'agrobot_bridge = agrobot_husky_nav.agrobot_bridge:main',
            'zed_stereo = agrobot_husky_nav.zed_stereo:main',
            'voxel_local_map = agrobot_husky_nav.voxel_local_map:main',
            'explore = agrobot_husky_nav.explore:main',
            'goto_point = agrobot_husky_nav.goto_point:main',
            'follow_me = agrobot_husky_nav.follow_me:main',
        ],
    },
)
