from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final_mission_robot'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woo',
    maintainer_email='woo@example.com',
    description='Final mission robot package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'air_clean_pure_controller = final_mission_robot.air_clean_pure_controller:main',
            'auto_initial_pose = final_mission_robot.auto_initial_pose:main',
            'mission_manager = final_mission_robot.mission_manager:main',
        ],
    },
)
