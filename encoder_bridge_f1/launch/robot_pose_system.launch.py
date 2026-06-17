from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_node = Node(
        package='encoder_bridge',
        executable='aruco_node',
        name='f1_aruco_node',
        output='screen',
        remappings=[
            ('/aruco_marker', '/f1/aruco_marker'),
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
            ('/target_marker', '/f1/target_marker'),
            ('/aruco_image/compressed', '/f1/aruco_image/compressed')
        ]
    )

    serial_encoder_node = Node(
        package='encoder_bridge',
        executable='serial_encoder_node',
        name='f1_serial_encoder_node',
        output='screen',
        remappings=[
            ('/robot_cmd', '/f1/robot_cmd'),
            ('/encoder_counts', '/f1/encoder_counts')
        ]
    )

    relative_pose_node = Node(
        package='encoder_bridge',
        executable='relative_pose_node',
        name='f1_relative_pose_node',
        output='screen',
        remappings=[
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
            ('/encoder_counts', '/f1/encoder_counts'),
            ('/relative_pose', '/f1/relative_pose')
        ]
    )

    waypoint_drive_node = Node(
        package='encoder_bridge',
        executable='waypoint_drive_node',
        name='f1_waypoint_drive_node',
        output='screen',
        remappings=[
            ('/relative_pose', '/f1/relative_pose'),
            ('/goal_pose', '/f1/goal_pose'),
            ('/aruco_marker', '/f1/aruco_marker'),
            ('/robot_cmd', '/f1/robot_cmd')
        ]
    )

    return LaunchDescription([
        aruco_node,
        serial_encoder_node,
        relative_pose_node,
        waypoint_drive_node
    ])
