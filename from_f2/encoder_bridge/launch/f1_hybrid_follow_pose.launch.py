# flake8: noqa: E501
"""F1 hybrid follow/recovery launch; do not run with legacy F1 controllers."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('encoder_bridge'), 'config'])
    calibration = PathJoinSubstitution([config_dir, 'camera_calibration.yaml'])
    marker_reference = PathJoinSubstitution([config_dir, 'aruco_reference.yaml'])

    aruco = Node(
        package='encoder_bridge',
        executable='aruco_node',
        name='f1_aruco_node',
        output='screen',
        parameters=[{
            'target_marker_id': 159,
            'marker_size_m': 0.050,
            'target_marker_size_m': 0.050,
            'camera_index': 0,
            'calib_path': calibration,
        }],
        remappings=[
            ('/aruco_marker', '/f1/aruco_marker'),
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
            ('/target_marker', '/f1/target_marker'),
            ('/aruco_image/compressed', '/f1/aruco_image/compressed'),
        ],
    )

    serial = Node(
        package='encoder_bridge',
        executable='serial_encoder_node',
        name='f1_serial_encoder_node',
        output='screen',
        remappings=[
            ('/robot_cmd', '/f1/robot_cmd'),
            ('/encoder_counts', '/f1/encoder_counts'),
        ],
    )

    relative_pose = Node(
        package='encoder_bridge',
        executable='relative_pose_node',
        name='f1_relative_pose_node',
        output='screen',
        parameters=[{'relative_pose_publish_interval_sec': 0.10}],
        remappings=[
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
            ('/encoder_counts', '/f1/encoder_counts'),
            ('/relative_pose', '/f1/relative_pose'),
            ('/localization_status', '/f1/localization_status'),
        ],
    )

    hybrid = Node(
        package='encoder_bridge',
        executable='f1_hybrid_follow_pose_node',
        name='f1_hybrid_follow_pose_node',
        output='screen',
        parameters=[{
            'target_marker_id': 159,
            'lost_timeout_sec': 0.70,
            'recovery_sample_count': 4,
            'recovery_min_markers': 3,
            'recovery_max_reproj_px': 2.0,
            'recovery_pos_spread_m': 0.05,
            'recovery_yaw_spread_deg': 5.0,
            'allow_two_marker_initial': False,
            'marker_reference_path': marker_reference,
            # Tune these two values for the physical corner geometry.
            'reacquire_forward_m': 0.25,
            'reacquire_yaw_offset_deg': 0.0,
            # Map start pose from config/aruco_reference.yaml.
            'home_x': -0.03757,
            'home_y': 0.10524,
            'home_yaw': -0.00715,
            # Disabled by default; use /f1/load_wait or /mission/load_wait.
            'load_wait_after_target_hold_sec': 0.0,
        }],
        remappings=[
            ('/target_marker', '/f1/target_marker'),
            ('/aruco_marker', '/f1/aruco_marker'),
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
            ('/relative_pose', '/f1/relative_pose'),
            ('/encoder_counts', '/f1/encoder_counts'),
            ('/robot_cmd', '/f1/robot_cmd'),
            ('/hybrid_status', '/f1/hybrid_status'),
            ('/load_done', '/f1/load_done'),
            ('/return_home', '/f1/return_home'),
            ('/load_wait', '/f1/load_wait'),
            ('/reanchor', '/f1/reanchor'),
        ],
    )

    return LaunchDescription([aruco, serial, relative_pose, hybrid])
