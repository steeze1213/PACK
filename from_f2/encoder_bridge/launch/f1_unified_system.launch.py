# flake8: noqa: E501
"""F1-only unified follow, waypoint, recovery, and return-home system.

Do not run this launch together with robot_pose_system.launch.py,
f1_follow_system.launch.py, or f1_hybrid_follow_pose.launch.py. They use the
same /dev/video0 and /dev/ttyACM0 devices and may publish competing commands.
This launch never starts F2 perception, camera, or serial nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_dir = PathJoinSubstitution([
        FindPackageShare('encoder_bridge'), 'config'
    ])
    calibration = PathJoinSubstitution([
        config_dir, 'camera_calibration.yaml'
    ])
    marker_reference = PathJoinSubstitution([
        config_dir, 'aruco_reference.yaml'
    ])

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
            ('/target_marker', '/f1/target_marker'),
            ('/aruco_marker', '/f1/aruco_marker'),
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
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
            'start_enabled': False,
            'marker_reference_path': marker_reference,
            'home_x': -0.03757,
            'home_y': 0.10524,
            'home_yaw': -0.00715,
            'load_wait_after_target_hold_sec': 0.0,
        }],
        remappings=[
            ('/target_marker', '/f1/target_marker'),
            ('/aruco_marker', '/f1/aruco_marker'),
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
            ('/relative_pose', '/f1/relative_pose'),
            ('/encoder_counts', '/f1/encoder_counts'),
            ('/robot_cmd', '/f1/follow_cmd'),
            ('/hybrid_status', '/f1/hybrid_status'),
            ('/load_wait', '/f1/load_wait'),
            ('/load_done', '/f1/load_done'),
            ('/return_home', '/f1/return_home'),
            ('/reanchor', '/f1/reanchor'),
            ('/mission_enable', '/f1/mission_enable'),
        ],
    )

    waypoint = Node(
        package='encoder_bridge',
        executable='waypoint_drive_node',
        name='f1_waypoint_drive_node',
        output='screen',
        parameters=[{
            'command_topic': '/f1/waypoint_cmd',
            'pose_stamped_goal_topic': '/f1/goal_pose',
            'localization_status_topic': '/f1/localization_status',
            'path_topic': '/f1/path_points',
            'path_only_mode': False,
            # F1's field version publishes legacy RELPOSE without the newer
            # pose_quality/localization_status fields. Freshness timeout and
            # the mode manager remain fail-closed if that stream stops.
            'allow_legacy_pose': True,
            'require_localization_status': False,
        }],
        remappings=[
            ('/relative_pose', '/f1/relative_pose'),
            # Preserve legacy String GOAL support without conflicting with
            # the dashboard's geometry_msgs/PoseStamped /f1/goal_pose.
            ('/goal_pose', '/f1/goal_pose_legacy'),
            ('/waypoint_status', '/f1/waypoint_status'),
        ],
    )

    mode_manager = Node(
        package='encoder_bridge',
        executable='f1_mode_manager_node',
        name='f1_mode_manager_node',
        output='screen',
    )

    return LaunchDescription([
        aruco,
        serial,
        relative_pose,
        hybrid,
        waypoint,
        mode_manager,
    ])
