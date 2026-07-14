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

    turtlebot_follow_node = Node(
        package='encoder_bridge',
        executable='turtlebot_follow_node',
        name='f1_turtlebot_follow_node',
        output='screen',
        remappings=[
            ('/relative_pose', '/f1/relative_pose'),
            ('/target_marker', '/f1/target_marker'),
            ('/follow_enable', '/f1/follow_enable'),
            ('/robot_cmd', '/f1/robot_cmd'),
            ('/follow_status', '/f1/follow_status')
        ]
    )

    return LaunchDescription([
        aruco_node,
        serial_encoder_node,
        relative_pose_node,
        turtlebot_follow_node
    ])
