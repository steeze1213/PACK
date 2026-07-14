from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'encoder_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_encoder_node = encoder_bridge.serial_encoder_node:main',
            'encoder_odom_node = encoder_bridge.encoder_odom_node:main',
            'aruco_node = encoder_bridge.aruco_node:main',
            'aruco_follow_node = encoder_bridge.aruco_follow_node:main',
            'relative_pose_node = encoder_bridge.relative_pose_node:main',
            'waypoint_drive_node = encoder_bridge.waypoint_drive_node:main',
            'turtlebot_follow_node = encoder_bridge.turtlebot_follow_node:main',
            'turtlebot_to_f1_follow_node = encoder_bridge.turtlebot_to_f1_follow_node:main',
            'f1_to_f2_follow_node = encoder_bridge.f1_to_f2_follow_node:main',
            'f1_hybrid_follow_pose_node = encoder_bridge.f1_hybrid_follow_pose_node:main',
            'mission_manager_node = encoder_bridge.mission_manager_node:main',
            'f1_mode_manager_node = encoder_bridge.f1_mode_manager_node:main',
		'tb3_marker159_f1_pose_node = encoder_bridge.tb3_marker159_f1_pose_node:main',
        ],
    },
)
