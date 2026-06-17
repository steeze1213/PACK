from launch import LaunchDescription
from launch_ros.actions import Node


ARUCO_PARAMS = {
    'target_marker_id': 97,
    'marker_size_m': 0.085,
    'target_marker_size_m': 0.020,
    'camera_index': 0,
}


FOLLOW_PARAMS = {
    'target_marker_id': 97,
    'follow_distance': 0.50,
    'map_follow_center_distance': 0.50,
    'marker_far_distance': 0.45,
    'marker_pulse_distance': 0.35,
    'marker_hold_min_distance': 0.27,
    'marker_hold_max_distance': 0.33,
    'marker_too_close_distance': 0.22,
    'marker_emergency_distance': 0.17,
    'map_arrive_distance': 0.15,
    'target_marker_timeout_sec': 0.60,
    'marker_pose_distance_max_error': 0.50,
    'marker_pose_mismatch_required_count': 3,
    'follower_ahead_stop_distance': 0.10,
    'follower_ahead_lateral_tolerance': 0.35,
    'follower_ahead_required_count': 3,
}


def generate_launch_description():
    aruco_node = Node(
        package='encoder_bridge',
        executable='aruco_node',
        name='f2_aruco_node',
        output='screen',
        parameters=[ARUCO_PARAMS],
        remappings=[
            ('/aruco_marker', '/f2/aruco_marker'),
            ('/aruco_multi_markers', '/f2/aruco_multi_markers'),
            ('/target_marker', '/f2/target_marker'),
            ('/aruco_image/compressed', '/f2/aruco_image/compressed')
        ]
    )

    serial_encoder_node = Node(
        package='encoder_bridge',
        executable='serial_encoder_node',
        name='f2_serial_encoder_node',
        output='screen',
        remappings=[
            ('/robot_cmd', '/f2/robot_cmd'),
            ('/encoder_counts', '/f2/encoder_counts')
        ]
    )

    relative_pose_node = Node(
        package='encoder_bridge',
        executable='relative_pose_node',
        name='f2_relative_pose_node',
        output='screen',
        remappings=[
            ('/aruco_multi_markers', '/f2/aruco_multi_markers'),
            ('/encoder_counts', '/f2/encoder_counts'),
            ('/relative_pose', '/f2/relative_pose')
        ]
    )

    follow_node = Node(
        package='encoder_bridge',
        executable='f1_to_f2_follow_node',
        name='f2_follow_node',
        output='screen',
        parameters=[FOLLOW_PARAMS],
        remappings=[
            ('/relative_pose', '/f2/relative_pose'),
            ('/target_marker', '/f2/target_marker'),
            ('/follow_enable', '/f2/follow_enable'),
            ('/robot_cmd', '/f2/robot_cmd'),
            ('/follow_status', '/f2/follow_status')
        ]
    )

    return LaunchDescription([
        aruco_node,
        serial_encoder_node,
        relative_pose_node,
        follow_node
    ])
