"""PC-side final mission coordination; robot hardware runs on each robot.

Run f1_unified_system.launch.py on F1. A separate F2 unified launch will be
added after F1 validation. This launch intentionally opens no camera or serial
device and must run on only one PC/Flask host.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encoder_bridge',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
        ),
    ])
