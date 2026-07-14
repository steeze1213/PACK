# flake8: noqa: E501
"""F1-only unified follow, waypoint, load-exit, TB3 marker pose fusion, and return-home system.

Do not run this launch together with robot_pose_system.launch.py,
f1_follow_system.launch.py, or f1_hybrid_follow_pose.launch.py. They use the
same /dev/video0 and /dev/ttyACM0 devices and may publish competing commands.
This launch never starts F2 perception, camera, or serial nodes.

Main rule:
- F1 follows TB3 rear marker 159.
- If marker 159 is visible, f1_hybrid_follow_pose_node uses direct marker follow.
- If marker 159 is lost, f1_hybrid_follow_pose_node can fall back to TB3 /amcl_pose
  and /f1/relative_pose map-follow.
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

    relative_pose_wall = Node(
        package='encoder_bridge',
        executable='relative_pose_node',
        name='f1_relative_pose_node',
        output='screen',
        parameters=[{
            'relative_pose_publish_interval_sec': 0.10,
        }],
        remappings=[
            ('/aruco_multi_markers', '/f1/aruco_multi_markers'),
            ('/encoder_counts', '/f1/encoder_counts'),
            ('/relative_pose', '/f1/relative_pose_wall'),
            ('/localization_status', '/f1/localization_status_wall'),
        ],
    )

    tb3_marker159_pose = Node(
        package='encoder_bridge',
        executable='tb3_marker159_f1_pose_node',
        name='tb3_marker159_f1_pose_node',
        output='screen',
        parameters=[{
            'target_marker_id': 159,

            # TB3 base_link 기준 159번 마커 위치.
            # x 음수 = TB3 뒤쪽.
            'tb3_marker_x': -0.16,
            'tb3_marker_y': 0.00,

            # F1 base 기준 카메라 위치.
            'f1_camera_x': 0.15,
            'f1_camera_y': 0.00,

            # 159가 일정 시간 이상 안 보이면 wall pose 사용.
            'target_timeout_sec': 0.70,

            # 최종 /f1/relative_pose 발행 주기.
            'publish_period_sec': 0.10,
        }],
        remappings=[
            ('/amcl_pose', '/amcl_pose'),
            ('/f1/aruco_marker', '/f1/aruco_marker'),
            ('/f1/relative_pose_wall', '/f1/relative_pose_wall'),
            ('/f1/relative_pose', '/f1/relative_pose'),
            ('/f1/localization_status', '/f1/localization_status'),
        ],
    )

    hybrid = Node(
        package='encoder_bridge',
        executable='f1_hybrid_follow_pose_node',
        name='f1_hybrid_follow_pose_node',
        output='screen',
        parameters=[{
            'target_marker_id': 159,

            # mission_manager가 /f1/mode follow를 보낼 때까지 대기.
            'start_enabled': False,

            'marker_reference_path': marker_reference,

            # Home pose.
            'home_x': -0.03757,
            'home_y': 0.10524,
            'home_yaw': -0.00715,

            # ------------------------------------------------------------
            # Marker follow safety parameters.
            # 적재를 위해 TB3 뒤 159 마커에 더 가까이 붙도록 조정.
            # ------------------------------------------------------------
            'lost_timeout_sec': 0.80,
            'target_fresh_timeout_sec': 0.45,

            # 12cm 안쪽이면 너무 가까우므로 정지.
            'follow_stop_distance_m': 0.12,

            # 20cm보다 멀 때 전진.
            'follow_forward_distance_m': 0.20,

            # 12cm보다 가까우면 중심 정렬 q/e도 금지.
            'follow_min_align_distance_m': 0.12,

            # 중심 정렬 민감도.
            'follow_bearing_tolerance_deg': 10.0,
            'follow_hard_bearing_deg': 35.0,

            # ------------------------------------------------------------
            # TB3 AMCL fallback parameters.
            # marker 159를 놓치면 /amcl_pose + /f1/relative_pose 기준으로
            # TB3 뒤쪽 목표점을 따라감.
            # ------------------------------------------------------------
            'tb3_follow_distance_m': 0.15,
            'tb3_pose_timeout_sec': 1.20,
            'f1_pose_timeout_sec': 1.20,
            'tb3_map_position_tolerance_m': 0.05,
            'tb3_map_yaw_tolerance_deg': 18.0,
            'tb3_map_bearing_tolerance_deg': 15.0,
            'tb3_map_hard_bearing_deg': 75.0,

            # Recovery fallback.
            'recovery_sample_count': 4,
            'recovery_min_markers': 3,
            'recovery_max_reproj_px': 2.0,
            'recovery_pos_spread_m': 0.05,
            'recovery_yaw_spread_deg': 5.0,
            'multi_pose_sync_sec': 0.35,
            'marker_group_adjacent_gap': 2,
            'allow_two_marker_initial': False,

            # Original reacquire behavior. TB3 map fallback이 안 될 때만 사용.
            'reacquire_forward_m': 0.15,
            'reacquire_yaw_offset_deg': 0.0,
            'reacquire_timeout_sec': 8.0,
            'goal_position_tolerance_m': 0.07,
            'goal_yaw_tolerance_deg': 8.0,
            'goal_bearing_tolerance_deg': 10.0,

            # ------------------------------------------------------------
            # Load hold.
            # 기존 0.30~0.45m는 로봇팔 적재 위치 기준으로 너무 멀었음.
            # 적재 시 TB3 뒤 159 마커 기준 12~20cm 근처에서 대기.
            # ------------------------------------------------------------
            'load_wait_after_target_hold_sec': 0.0,
            'load_hold_min_distance_m': 0.12,
            'load_hold_max_distance_m': 0.20,
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
            ('/amcl_pose', '/amcl_pose'),
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
            'allow_legacy_pose': True,
            'require_localization_status': False,
        }],
        remappings=[
            ('/relative_pose', '/f1/relative_pose'),
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
        relative_pose_wall,
        tb3_marker159_pose,
        hybrid,
        waypoint,
        mode_manager,
    ])
