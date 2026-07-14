# flake8: noqa: E501
"""F2-only unified follow, waypoint, and A* path system.

Run this on the F2 Raspberry Pi only. It never starts F1 perception, camera, or
serial nodes. The only node allowed to publish /f2/robot_cmd is
f2_mode_manager_node.

Marker rule:
- TB3 rear marker = 159
- F1 rear marker = 158
- F2 default follow target = 158
- F2 can switch target at runtime by /f2/target_marker_cmd

Runtime follow target:
- /f2/target_marker_cmd = 158 -> F2 follows F1
- /f2/target_marker_cmd = 159 -> F2 follows TB3

Localization rule:
- relative_pose_node publishes wall-marker pose to /f2/relative_pose_wall
- tb3_marker159_f2_pose_node publishes final pose to /f2/relative_pose

Waypoint/A* rule:
- mission_manager or manual command publishes /f2/astar_goal
- f2_map_astar_planner_node makes /f2/path_points
- waypoint_drive_node follows /f2/path_points and publishes /f2/waypoint_cmd
- f2_mode_manager_node selects /f2/waypoint_cmd or /f2/follow_cmd and publishes /f2/robot_cmd
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

    aruco = Node(
        package='encoder_bridge',
        executable='aruco_node',
        name='f2_aruco_node',
        output='screen',
        parameters=[{
            # F2 default target is the rear marker on F1.
            # F1 rear marker = 158
            # TB3 rear marker = 159
            #
            # Runtime switch:
            #   /f2/target_marker_cmd = 158
            #   /f2/target_marker_cmd = 159
            'target_marker_id': 158,

            # Current field markers are 5 cm.
            'marker_size_m': 0.050,
            'target_marker_size_m': 0.050,

            # F2 camera.
            # /dev/video0 is Video Capture.
            # /dev/video1 is Metadata Capture and must not be used.
            'camera_index': 0,

            'calib_path': calibration,
        }],
        remappings=[
            ('/target_marker_cmd', '/f2/target_marker_cmd'),
            ('/target_marker', '/f2/target_marker'),
            ('/aruco_marker', '/f2/aruco_marker'),
            ('/aruco_multi_markers', '/f2/aruco_multi_markers'),
            ('/aruco_image/compressed', '/f2/aruco_image/compressed'),
        ],
    )

    serial = Node(
        package='encoder_bridge',
        executable='serial_encoder_node',
        name='f2_serial_encoder_node',
        output='screen',
        remappings=[
            ('/robot_cmd', '/f2/robot_cmd'),
            ('/encoder_counts', '/f2/encoder_counts'),
        ],
    )

    relative_pose = Node(
        package='encoder_bridge',
        executable='relative_pose_node',
        name='f2_relative_pose_node',
        output='screen',
        parameters=[{
            'relative_pose_publish_interval_sec': 0.10,
            'demo_stable_pose_mode': False,
        }],
        remappings=[
            ('/aruco_multi_markers', '/f2/aruco_multi_markers'),
            ('/encoder_counts', '/f2/encoder_counts'),
            ('/relative_pose', '/f2/relative_pose_wall'),
            ('/localization_status', '/f2/localization_status_wall'),
        ],
    )

    tb3_marker159_pose = Node(
        package='encoder_bridge',
        executable='tb3_marker159_f2_pose_node',
        name='tb3_marker159_f2_pose_node',
        output='screen',
        parameters=[{
            'target_marker_id': 159,

            # TB3 base_link 기준 159번 마커 위치.
            # x 음수 = TB3 뒤쪽.
            'tb3_marker_x': -0.16,
            'tb3_marker_y': 0.0,

            # F2 base 기준 카메라 위치.
            'f2_camera_x': 0.15,
            'f2_camera_y': 0.0,

            # 159가 일정 시간 이상 안 보이면 wall pose fallback.
            'target_timeout_sec': 0.90,

            # 최종 /f2/relative_pose 발행 주기.
            'publish_period_sec': 0.10,
        }],
        remappings=[
            ('/amcl_pose', '/amcl_pose'),
            ('/f2/aruco_marker', '/f2/aruco_marker'),
            ('/f2/relative_pose_wall', '/f2/relative_pose_wall'),
            ('/f2/relative_pose', '/f2/relative_pose'),
            ('/f2/localization_status', '/f2/localization_status'),
        ],
    )

    astar = Node(
        package='encoder_bridge',
        executable='f2_map_astar_planner_node',
        name='f2_map_astar_planner_node',
        output='screen',
        parameters=[{
            # F2 안에 이 파일들이 있어야 함:
            # /home/f2/map_cleaned.yaml
            # /home/f2/map_cleaned.pgm
            'map_yaml_path': '/home/f2/map_cleaned.yaml',

            # A* output.
            # waypoint_drive_node가 /f2/path_points를 구독함.
            'path_topic': '/f2/path_points',
            'path_vis_topic': '/f2/planned_path',

            # F2 current pose.
            'relative_pose_topic': '/f2/relative_pose',

            # A* input goal.
            # /f2/goal_pose로 직접 보내면 직선 주행이 되므로,
            # A*용 목표는 /f2/astar_goal로 분리.
            'astar_goal_topic': '/f2/astar_goal',

            # Obstacle inflation.
            # 너무 크게 잡으면 경로가 안 나오고,
            # 너무 작게 잡으면 벽에 붙음.
            'robot_radius_m': 0.18,
            'safety_margin_m': 0.08,
            'unknown_is_obstacle': True,

            # Path output.
            'waypoint_spacing_m': 0.12,
            'max_waypoints': 100,
            'allow_diagonal': True,

            # If start/goal is inside inflated obstacle, search nearby free cell.
            'goal_search_radius_m': 1.80,
            'start_search_radius_m': 3.00,
        }],
    )

    follow = Node(
        package='encoder_bridge',
        executable='f1_to_f2_follow_node',
        name='f2_follow_node',
        output='screen',
        parameters=[{
            # Default:
            #   158 = follow F1
            # Runtime:
            #   /f2/target_marker_cmd = 158 -> active_leader=F1
            #   /f2/target_marker_cmd = 159 -> active_leader=TB3
            'target_marker_id': 158,

            # ------------------------------------------------------------
            # Follow distances.
            # F2는 벽에 박는 문제가 있었으므로 12~17cm 세팅은 너무 위험.
            # 18~25cm 정도에서 HOLD 하도록 안전하게 조정.
            # ------------------------------------------------------------
            'follow_distance': 0.24,
            'map_follow_center_distance': 0.28,

            # 필요하면 실제 마커/카메라 부착 위치 보정용.
            'leader_pose_to_rear_marker_offset': 0.0,
            'follower_pose_to_front_camera_offset': 0.0,
            'desired_bumper_gap': -1.0,
            'desired_camera_marker_distance': 0.0,

            # ------------------------------------------------------------
            # Marker distance gates.
            # 18~25cm 사이면 HOLD.
            # 16cm 이하면 too close.
            # 13cm 이하면 emergency.
            # ------------------------------------------------------------
            'marker_use_max_distance': 1.50,
            'marker_far_distance': 0.40,
            'marker_pulse_distance': 0.28,
            'marker_hold_min_distance': 0.18,
            'marker_hold_max_distance': 0.25,
            'marker_too_close_distance': 0.16,
            'marker_emergency_distance': 0.13,

            # ------------------------------------------------------------
            # Marker angle gates.
            # ------------------------------------------------------------
            'marker_align_tolerance_deg': 10.0,
            'marker_forward_limit_deg': 18.0,
            'marker_hard_stop_angle_deg': 25.0,

            # ------------------------------------------------------------
            # Map follow gates.
            # F2가 marker를 잃었을 때 map follow로 벽에 밀고 들어가는 문제 방지.
            # A* 이동은 waypoint 모드가 담당하고,
            # follow 모드에서는 map forward를 거의 사용하지 않게 함.
            # ------------------------------------------------------------
            'map_arrive_distance': 0.30,
            'map_yaw_tolerance_deg': 35.0,
            'map_hard_stop_angle_deg': 90.0,

            # ------------------------------------------------------------
            # Filter and timeout.
            # ------------------------------------------------------------
            'marker_distance_filter_alpha': 0.35,
            'target_marker_timeout_sec': 1.00,
            'follower_pose_timeout_sec': 1.30,
            'leader_pose_timeout_sec': 1.50,

            # ------------------------------------------------------------
            # Pose consistency / mode transition.
            # ------------------------------------------------------------
            'marker_pose_distance_max_error': 0.50,
            'marker_pose_mismatch_required_count': 3,
            'mode_transition_stop_sec': 0.15,

            # ------------------------------------------------------------
            # Follower position safety.
            # F2가 리더보다 앞에 있다고 판단되면 정지.
            # ------------------------------------------------------------
            'follower_ahead_stop_distance': 0.10,
            'follower_ahead_lateral_tolerance': 0.35,
            'follower_ahead_required_count': 3,

            # ------------------------------------------------------------
            # Leader moving detection.
            # ------------------------------------------------------------
            'leader_moving_speed': 0.015,
            'leader_turning_speed': 0.05,

            # ------------------------------------------------------------
            # Command pulse parameters.
            # 벽 충돌 방지를 위해 forward pulse를 줄임.
            # ------------------------------------------------------------
            'marker_align_pulse_sec': 0.12,
            'marker_far_forward_pulse_sec': 0.25,
            'marker_near_forward_pulse_sec': 0.08,
            'marker_moving_forward_pulse_sec': 0.06,
            'map_pivot_pulse_sec': 0.08,
            'map_forward_pulse_sec': 0.04,
            'command_stop_pause_sec': 0.08,
        }],
        remappings=[
            # F2 own pose.
            ('/relative_pose', '/f2/relative_pose'),

            # F1 leader pose. Used when target_marker_id=158.
            ('/f1/relative_pose', '/f1/relative_pose'),

            # TB3 leader pose. Used when target_marker_id=159.
            ('/amcl_pose', '/amcl_pose'),

            # Target marker detection output from aruco_node.
            ('/target_marker', '/f2/target_marker'),

            # Runtime target command shared with aruco_node and follow_node.
            ('/target_marker_cmd', '/f2/target_marker_cmd'),

            ('/follow_enable', '/f2/follow_enable'),

            # Do not publish directly to /f2/robot_cmd.
            # The mode manager is the only /f2/robot_cmd publisher.
            ('/robot_cmd', '/f2/follow_cmd'),
            ('/follow_status', '/f2/follow_status'),
        ],
    )

    waypoint = Node(
        package='encoder_bridge',
        executable='waypoint_drive_node',
        name='f2_waypoint_drive_node',
        output='screen',
        parameters=[{
            'command_topic': '/f2/waypoint_cmd',

            # /f2/goal_pose는 직접 목표용으로 남겨둠.
            # 하지만 장애물 회피하려면 /f2/astar_goal -> /f2/path_points 흐름을 사용.
            'pose_stamped_goal_topic': '/f2/goal_pose',

            'localization_status_topic': '/f2/localization_status',
            'path_topic': '/f2/path_points',

            # A* path가 있으면 path_points를 따라감.
            # False로 둬야 기존 직접 goal 테스트도 가능.
            'path_only_mode': False,

            'allow_legacy_pose': True,
            'require_localization_status': False,
        }],
        remappings=[
            ('/relative_pose', '/f2/relative_pose'),
            ('/goal_pose', '/f2/goal_pose_legacy'),
            ('/waypoint_status', '/f2/waypoint_status'),
        ],
    )

    mode_manager = Node(
        package='encoder_bridge',
        executable='f2_mode_manager_node',
        name='f2_mode_manager_node',
        output='screen',
    )

    return LaunchDescription([
        aruco,
        serial,
        relative_pose,
        tb3_marker159_pose,
        astar,
        follow,
        waypoint,
        mode_manager,
    ])
