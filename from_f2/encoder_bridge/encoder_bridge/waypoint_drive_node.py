import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class WaypointDriveNode(Node):
    def __init__(self):
        super().__init__('waypoint_drive_node')

        self.pose_sub = self.create_subscription(
            String,
            '/f2/relative_pose',
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/f2/goal_pose',
            self.goal_pose_callback,
            10
        )

        self.goal_string_sub = self.create_subscription(
            String,
            '/f2/goal_cmd',
            self.goal_string_callback,
            10
        )

        self.path_sub = self.create_subscription(
            String,
            '/f2/path_points',
            self.path_callback,
            10
        )

        self.aruco_sub = self.create_subscription(
            String,
            '/f2/aruco_marker',
            self.aruco_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/f2/waypoint_cmd',
            10
        )

        self.goal_x = None
        self.goal_y = None
        self.final_goal_x = None
        self.final_goal_y = None

        self.path_points = []
        self.path_index = 0
        self.using_path = False
        self.waiting_for_astar_path = False
        # A* path is optional by default.
        # If /f2/path_points publishes PATH,status=ok, this node follows that path.
        # If no path is published, it drives directly to the clicked PoseStamped goal.
        self.astar_path_required = False
        self.path_target_lookahead_m = 0.70
        self.path_waypoint_reach_m = 0.45
        self.intermediate_waypoint_reach_m = 0.45
        self.waypoint_skip_min_dist_m = 0.55
        self.waypoint_skip_increase_m = 0.15
        self.waypoint_skip_yaw_deg = 110.0
        self.current_target_min_distance = None
        self.current_target_last_distance = None
        self.pivot_escape_path_index = -999
        self.pivot_escape_count = 0
        self.force_forward_start_time = None

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.marker_seen = 0
        self.aruco_accepted = 0
        self.raw_marker_detected = False
        self.source = 'none'

        self.last_marker_id = -1
        self.last_marker_offset_x = 0.0

        self.last_pose_time = None
        self.last_aruco_time = None
        self.marker_lost_start_time = None
        self.accepted_pose_lost_start_time = None

        self.pose_timeout_sec = 0.60
        self.aruco_timeout_sec = 0.40
        self.blind_grace_sec = 2.00
        self.accepted_pose_wait_sec = 3.0
        self.marker_missing_search_sec = 3.00
        self.accepted_pose_streak = 0
        self.last_accepted_pose_time = None

        self.reacquire_required_count = 1
        self.reacquire_count = 0
        self.reacquire_stop_sec = 0.30
        self.reacquire_start_time = None

        self.arrive_distance = 0.18
        self.near_goal_distance = 0.35

        self.yaw_tolerance_deg = 50.0
        self.hard_stop_angle_deg = 179.0

        self.pivot_pulse_sec = 0.14
        self.pivot_start_yaw_deg = 55.0
        self.current_pivot_pulse_sec = self.pivot_pulse_sec
        self.after_pivot_stop_sec = 0.25
        self.pivot_escape_limit = 8
        self.pivot_escape_forward_sec = 0.35
        self.blind_yaw_limit_deg = 55.0
        self.near_goal_yaw_forward_limit_deg = 42.0
        self.unstable_forward_yaw_limit_deg = 50.0

        self.search_pivot_sec = 0.08
        self.search_stop_sec = 0.60
        self.search_pause_sec = 1.00
        self.search_max_active_sec = 4.0

        self.search_direction = 'q'
        self.search_phase = 'STOP'
        self.search_phase_start_time = None
        self.search_start_time = None
        self.search_pause_start_time = None

        self.search_step_count = 0
        self.search_step_limit = 3

        self.control_interval = 0.05
        self.timer = self.create_timer(
            self.control_interval,
            self.control_loop
        )

        self.arrived = False
        self.no_goal_logged = False

        self.mode = 'IDLE'

        self.pivot_cmd = None
        self.pivot_start_time = None
        self.pivot_stop_time = None

        self.recovery_mode = 'WAITING_FOR_MARKER'

        self.last_cmd = 's'
        self.last_log_time = self.get_clock().now()
        self.last_warning_time = self.get_clock().now()

        self.get_logger().info('waypoint_drive_node F2 VARIABLE_PIVOT_ASTAR_OPTIONAL_V5_FIXED_TOPICS started.')
        self.get_logger().info(
            'Mode: PoseStamped direct goal + optional A* path + lookahead + variable pivot + waypoint_cmd output'
        )
        self.get_logger().info(
            'Subscribing: /f2/relative_pose, /f2/goal_pose, /f2/goal_cmd, /f2/path_points, /f2/aruco_marker'
        )
        self.get_logger().info('Publishing: /f2/waypoint_cmd')
        self.get_logger().info(
            f'arrive_distance={self.arrive_distance}, '
            f'near_goal_distance={self.near_goal_distance}, '
            f'yaw_tolerance_deg={self.yaw_tolerance_deg}, '
            f'blind_grace_sec={self.blind_grace_sec}, '
            f'accepted_pose_wait_sec={self.accepted_pose_wait_sec}, '
            f'marker_missing_search_sec={self.marker_missing_search_sec}, '
            f'reacquire_required_count={self.reacquire_required_count}, '
            f'search_pivot_sec={self.search_pivot_sec}, '
            f'search_stop_sec={self.search_stop_sec}, '
            f'astar_path_required={self.astar_path_required}, '
            f'path_target_lookahead_m={self.path_target_lookahead_m}, '
            f'path_waypoint_reach_m={self.path_waypoint_reach_m}, '
            f'pivot_start_yaw_deg={self.pivot_start_yaw_deg}, '
            f'waypoint_skip_yaw_deg={self.waypoint_skip_yaw_deg}, '
            f'commands=w/q/e/s only'
        )
        self.get_logger().info('Waiting for PC map click goal...')

    def pose_callback(self, msg):
        parsed = self.parse_key_value_message(
            msg.data,
            'RELPOSE'
        )

        if parsed is None:
            return

        try:
            self.current_x = float(parsed.get('x'))
            self.current_y = float(parsed.get('y'))
            self.current_yaw = float(parsed.get('yaw'))
            self.marker_seen = int(
                parsed.get('marker_seen', 0)
            )
            self.aruco_accepted = int(
                parsed.get('aruco_accepted', 0)
            )
            self.source = str(
                parsed.get('source', 'unknown')
            )
        except (TypeError, ValueError):
            return

        self.last_pose_time = self.get_clock().now()

        if (
            self.marker_seen == 1
            and self.aruco_accepted == 1
        ):
            self.accepted_pose_streak += 1
            self.last_accepted_pose_time = self.last_pose_time
            self.accepted_pose_lost_start_time = None
        else:
            self.accepted_pose_streak = 0

    def aruco_callback(self, msg):
        parsed = self.parse_key_value_message(
            msg.data,
            'ARUCO'
        )

        if parsed is None:
            return

        now = self.get_clock().now()
        self.last_aruco_time = now

        try:
            detected = int(
                parsed.get('detected', 0)
            )
        except (TypeError, ValueError):
            detected = 0

        if detected == 1:
            self.raw_marker_detected = True
            self.marker_lost_start_time = None

            try:
                self.last_marker_id = int(
                    parsed.get('id', -1)
                )
            except (TypeError, ValueError):
                self.last_marker_id = -1

            try:
                self.last_marker_offset_x = float(
                    parsed.get('offset_x', 0.0)
                )
            except (TypeError, ValueError):
                self.last_marker_offset_x = 0.0

            self.reacquire_count += 1
        else:
            self.raw_marker_detected = False
            self.reacquire_count = 0

            if self.marker_lost_start_time is None:
                self.marker_lost_start_time = now

    def goal_pose_callback(self, msg):
        try:
            self.goal_x = float(msg.pose.position.x)
            self.goal_y = float(msg.pose.position.y)
            self.final_goal_x = self.goal_x
            self.final_goal_y = self.goal_y

            self.path_points = []
            self.path_index = 0
            self.using_path = False
            self.waiting_for_astar_path = self.astar_path_required

            self.arrived = False
            self.no_goal_logged = False

            self.reset_motion_state()
            self.reset_search_state()
            self.reset_target_tracking()

            if self.is_control_pose_usable():
                self.recovery_mode = 'NORMAL'
            else:
                self.recovery_mode = 'WAITING_FOR_MARKER'

            self.publish_cmd('s')

            if self.waiting_for_astar_path:
                self.get_logger().info(
                    f'Direct PoseStamped goal received, waiting for A* path: '
                    f'({self.goal_x:.3f}, {self.goal_y:.3f})'
                )
            else:
                self.get_logger().info(
                    f'Direct PoseStamped goal received: '
                    f'({self.goal_x:.3f}, {self.goal_y:.3f})'
                )

        except Exception as e:
            self.get_logger().warn(f'Invalid PoseStamped goal ignored: {e}')

    def goal_string_callback(self, msg):
        data = msg.data.strip()

        if data == 'GOAL_CLEAR':
            self.clear_goal()
            self.get_logger().info('Goal cleared. Stop.')
            return

        parsed = self.parse_key_value_message(
            data,
            'GOAL'
        )

        if parsed is None:
            return

        try:
            self.goal_x = float(parsed.get('x'))
            self.goal_y = float(parsed.get('y'))
            self.final_goal_x = self.goal_x
            self.final_goal_y = self.goal_y

            self.path_points = []
            self.path_index = 0
            self.using_path = False
            self.waiting_for_astar_path = self.astar_path_required

            self.arrived = False
            self.no_goal_logged = False

            self.reset_motion_state()
            self.reset_search_state()

            if self.is_control_pose_usable():
                self.recovery_mode = 'NORMAL'
            else:
                self.recovery_mode = 'WAITING_FOR_MARKER'

            self.publish_cmd('s')

            self.get_logger().info(
                f'New string goal received: '
                f'x={self.goal_x:.3f}, '
                f'y={self.goal_y:.3f}'
            )

        except (TypeError, ValueError):
            self.get_logger().warn(
                f'Invalid goal message: {data}'
            )

    def path_callback(self, msg):
        data = msg.data.strip()

        parsed = self.parse_key_value_message(
            data,
            'PATH'
        )

        if parsed is None:
            return

        status = str(parsed.get('status', ''))

        if status != 'ok':
            reason = str(parsed.get('reason', 'unknown'))
            self.path_points = []
            self.path_index = 0
            self.using_path = False
            self.waiting_for_astar_path = False
            self.arrived = False
            self.reset_motion_state()
            self.reset_search_state()
            self.publish_cmd('s')
            self.get_logger().warn(
                f'A* path failed or invalid: reason={reason}. Stop and wait for a new goal.'
            )
            self.goal_x = None
            self.goal_y = None
            self.final_goal_x = None
            self.final_goal_y = None
            self.no_goal_logged = False
            return

        try:
            count = int(parsed.get('count', 0))
        except (TypeError, ValueError):
            count = 0

        points = []

        for index in range(count):
            try:
                x = float(parsed.get(f'p{index}_x'))
                y = float(parsed.get(f'p{index}_y'))
                points.append((x, y))
            except (TypeError, ValueError):
                continue

        if len(points) < 2:
            self.path_points = []
            self.path_index = 0
            self.using_path = False
            self.waiting_for_astar_path = False
            self.publish_cmd('s')
            self.get_logger().warn(
                f'A* path ignored: not enough points. Stop. count={len(points)}'
            )
            self.goal_x = None
            self.goal_y = None
            self.final_goal_x = None
            self.final_goal_y = None
            self.no_goal_logged = False
            return

        self.path_points = points
        self.path_index = 0
        self.using_path = True
        self.waiting_for_astar_path = False
        self.arrived = False
        self.no_goal_logged = False

        self.final_goal_x = points[-1][0]
        self.final_goal_y = points[-1][1]

        self.select_path_target()
        self.reset_target_tracking()

        self.reset_motion_state()
        self.reset_search_state()

        if self.is_control_pose_usable():
            self.recovery_mode = 'NORMAL'
        else:
            self.recovery_mode = 'WAITING_FOR_MARKER'

        self.publish_cmd('s')

        self.get_logger().info(
            f'A* path received: count={len(points)}, '
            f'active_index={self.path_index}, '
            f'active_goal=({self.goal_x:.3f},{self.goal_y:.3f}), '
            f'final_goal=({self.final_goal_x:.3f},{self.final_goal_y:.3f})'
        )

    def clear_goal(self):
        self.goal_x = None
        self.goal_y = None
        self.final_goal_x = None
        self.final_goal_y = None
        self.path_points = []
        self.path_index = 0
        self.using_path = False
        self.waiting_for_astar_path = False
        self.arrived = False

        self.reset_motion_state()
        self.reset_search_state()

        self.recovery_mode = 'WAITING_FOR_MARKER'
        self.publish_cmd('s')

    def reset_motion_state(self):
        self.mode = 'IDLE'

        self.pivot_cmd = None
        self.pivot_start_time = None
        self.pivot_stop_time = None
        self.current_pivot_pulse_sec = self.pivot_pulse_sec

    def reset_search_state(self):
        self.search_phase = 'STOP'
        self.search_phase_start_time = None
        self.search_start_time = None
        self.search_pause_start_time = None

        self.search_step_count = 0
        self.search_step_limit = 3

        self.reacquire_start_time = None
        self.reacquire_count = 0
        self.accepted_pose_lost_start_time = None
        self.accepted_pose_streak = 0
        self.force_forward_start_time = None
        self.pivot_escape_path_index = -999
        self.pivot_escape_count = 0

    def control_loop(self):
        now = self.get_clock().now()

        if self.goal_x is None or self.goal_y is None:
            if not self.no_goal_logged:
                self.get_logger().info('No goal. Stop.')
                self.no_goal_logged = True

            self.publish_cmd('s')
            return

        if self.arrived:
            self.publish_cmd('s')
            return

        if self.waiting_for_astar_path:
            self.reset_motion_state()
            self.publish_cmd('s')
            self.log_warning_throttled(
                'Waiting for A* path. Hold stop before driving.'
            )
            return

        if not self.is_pose_fresh(now):
            self.reset_motion_state()
            self.publish_cmd('s')
            self.log_warning_throttled(
                'Relative pose timeout. Stop for safety.'
            )
            return

        marker_detected = self.is_control_pose_usable()

        if marker_detected:
            self.handle_marker_detected(now)
        else:
            self.handle_marker_lost(now)

        if self.recovery_mode == 'SEARCH':
            self.handle_search_mode(now)
            return

        if self.recovery_mode == 'SEARCH_PAUSE':
            self.handle_search_pause(now)
            return

        if self.recovery_mode == 'REACQUIRE':
            self.handle_reacquire_mode(now)
            return

        if self.recovery_mode == 'WAITING_ACCEPTED_POSE':
            self.handle_waiting_accepted_pose(now)
            return

        if self.recovery_mode == 'WAITING_FOR_MARKER':
            self.handle_waiting_for_marker(now)
            return

        if self.mode == 'PIVOTING':
            self.handle_pivoting(now)
            return

        if self.mode == 'AFTER_PIVOT_STOP':
            self.handle_after_pivot_stop(now)
            return

        self.run_waypoint_control(now)

    def handle_marker_detected(self, now):
        self.accepted_pose_lost_start_time = None
        self.marker_lost_start_time = None

        if self.accepted_pose_streak > self.reacquire_count:
            self.reacquire_count = self.accepted_pose_streak

        if self.recovery_mode in [
            'SEARCH',
            'SEARCH_PAUSE',
            'WAITING_FOR_MARKER',
            'WAITING_ACCEPTED_POSE'
        ]:
            if self.accepted_pose_streak >= self.reacquire_required_count:
                self.enter_reacquire_mode(now)

            return

        if self.recovery_mode == 'REACQUIRE':
            return

        self.recovery_mode = 'NORMAL'


    def handle_marker_lost(self, now):
        if self.accepted_pose_lost_start_time is None:
            self.accepted_pose_lost_start_time = now

        if self.marker_lost_start_time is None:
            self.marker_lost_start_time = now

        self.reset_motion_state()

        if self.should_continue_when_pose_unstable(now):
            self.recovery_mode = 'WAITING_ACCEPTED_POSE'
            self.publish_cmd('w')
            self.log_warning_throttled(
                f'Accepted ArUco pose unstable, but marker/path remain valid. '
                f'Continue no-stuck forward grace.'
            )
            return

        if self.can_use_blind_grace(now):
            self.recovery_mode = 'WAITING_ACCEPTED_POSE'
            self.publish_cmd('w')
            self.log_warning_throttled(
                f'Accepted ArUco pose briefly unavailable. '
                f'Continue blind forward grace for up to '
                f'{self.blind_grace_sec:.2f}s.'
            )
            return

        self.publish_cmd('s')

        accepted_lost_duration = (
            now - self.accepted_pose_lost_start_time
        ).nanoseconds / 1e9

        marker_missing_duration = (
            now - self.marker_lost_start_time
        ).nanoseconds / 1e9

        marker_still_visible = (
            self.marker_seen == 1
            or self.is_marker_currently_detected()
        )

        if self.recovery_mode in [
            'SEARCH',
            'SEARCH_PAUSE'
        ]:
            return

        if marker_still_visible:
            if accepted_lost_duration >= self.accepted_pose_wait_sec:
                self.log_warning_throttled(
                    f'Marker visible but accepted pose stayed unstable for '
                    f'{accepted_lost_duration:.2f}s. Enter search pivot instead of permanent stop.'
                )
                self.enter_search_mode(now)
                return

            if self.recovery_mode != 'WAITING_ACCEPTED_POSE':
                self.recovery_mode = 'WAITING_ACCEPTED_POSE'

            self.log_warning_throttled(
                f'Marker is visible but accepted pose is unstable. '
                f'Hold stop and wait for relative pose recovery. '
                f'accepted_lost={accepted_lost_duration:.2f}s'
            )
            return

        if marker_missing_duration < self.marker_missing_search_sec:
            if self.recovery_mode != 'WAITING_FOR_MARKER':
                self.recovery_mode = 'WAITING_FOR_MARKER'

            self.log_warning_throttled(
                f'Marker not visible. Stop and wait '
                f'{marker_missing_duration:.2f}/'
                f'{self.marker_missing_search_sec:.2f}s before search.'
            )
            return

        if accepted_lost_duration < self.accepted_pose_wait_sec:
            if self.recovery_mode != 'WAITING_FOR_MARKER':
                self.recovery_mode = 'WAITING_FOR_MARKER'

            self.log_warning_throttled(
                f'Accepted ArUco pose unavailable. '
                f'Stop and wait {accepted_lost_duration:.2f}/'
                f'{self.accepted_pose_wait_sec:.2f}s before search.'
            )
            return

        if self.recovery_mode == 'REACQUIRE':
            self.get_logger().warn(
                'Accepted ArUco pose lost during reacquire. Return to waiting.'
            )

        self.enter_search_mode(now)


    def handle_waiting_accepted_pose(self, now):
        if self.is_control_pose_usable():
            self.handle_marker_detected(now)
            return

        if self.accepted_pose_lost_start_time is None:
            self.accepted_pose_lost_start_time = now

        if self.marker_lost_start_time is None:
            self.marker_lost_start_time = now

        accepted_lost_duration = (
            now - self.accepted_pose_lost_start_time
        ).nanoseconds / 1e9

        marker_still_visible = (
            self.marker_seen == 1
            or self.is_marker_currently_detected()
        )

        if self.should_continue_when_pose_unstable(now):
            self.publish_cmd('w')
            self.log_warning_throttled(
                f'Accepted pose unstable but marker is visible. '
                f'Keep moving to avoid stop lock '
                f'{accepted_lost_duration:.2f}/'
                f'{self.accepted_pose_wait_sec:.2f}s.'
            )
            return

        if self.can_use_blind_grace(now):
            self.publish_cmd('w')
            self.log_warning_throttled(
                f'Accepted pose briefly unstable. '
                f'Continue forward grace '
                f'{accepted_lost_duration:.2f}/'
                f'{self.blind_grace_sec:.2f}s.'
            )
            return

        self.reset_motion_state()
        self.publish_cmd('s')

        if marker_still_visible:
            if accepted_lost_duration >= self.accepted_pose_wait_sec:
                self.log_warning_throttled(
                    f'Marker visible but accepted pose not stable for '
                    f'{accepted_lost_duration:.2f}s. Enter search pivot.'
                )
                self.enter_search_mode(now)
                return

            self.log_warning_throttled(
                f'Marker visible but accepted pose not stable. '
                f'Hold stop and wait for relative pose recovery. accepted_lost='
                f'{accepted_lost_duration:.2f}s'
            )
            return

        marker_missing_duration = (
            now - self.marker_lost_start_time
        ).nanoseconds / 1e9

        if marker_missing_duration < self.marker_missing_search_sec:
            self.recovery_mode = 'WAITING_FOR_MARKER'
            self.log_warning_throttled(
                f'Marker missing. Stop and wait '
                f'{marker_missing_duration:.2f}/'
                f'{self.marker_missing_search_sec:.2f}s before search.'
            )
            return

        if accepted_lost_duration < self.accepted_pose_wait_sec:
            self.recovery_mode = 'WAITING_FOR_MARKER'
            self.log_warning_throttled(
                f'Accepted pose unavailable. Stop and wait '
                f'{accepted_lost_duration:.2f}/'
                f'{self.accepted_pose_wait_sec:.2f}s before search.'
            )
            return

        self.enter_search_mode(now)

    def handle_waiting_for_marker(self, now):
        if self.should_continue_when_pose_unstable(now):
            self.publish_cmd('w')
            self.log_warning_throttled(
                'Waiting marker/accepted pose, but marker is visible and path exists. Keep moving.'
            )
            return

        self.publish_cmd('s')

        if self.is_control_pose_usable():
            self.handle_marker_detected(now)
            return

        marker_still_visible = (
            self.marker_seen == 1
            or self.is_marker_currently_detected()
        )

        if marker_still_visible:
            if self.accepted_pose_lost_start_time is None:
                self.accepted_pose_lost_start_time = now

            elapsed = (
                now - self.accepted_pose_lost_start_time
            ).nanoseconds / 1e9

            self.recovery_mode = 'WAITING_ACCEPTED_POSE'

            self.log_warning_throttled(
                f'Marker visible but accepted pose not stable. '
                f'Wait without search. elapsed={elapsed:.2f}s'
            )
            return

        if self.marker_lost_start_time is None:
            self.marker_lost_start_time = now

        missing_duration = (
            now - self.marker_lost_start_time
        ).nanoseconds / 1e9

        if missing_duration < self.marker_missing_search_sec:
            self.log_warning_throttled(
                f'Marker missing. Wait '
                f'{missing_duration:.2f}/'
                f'{self.marker_missing_search_sec:.2f}s before search.'
            )
            return

        self.enter_search_mode(now)

    def can_use_blind_grace(self, now):
        if self.blind_grace_sec <= 0.0:
            return False

        if self.last_accepted_pose_time is None:
            return False

        elapsed = (
            now - self.last_accepted_pose_time
        ).nanoseconds / 1e9

        if elapsed > self.blind_grace_sec:
            return False

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        if self.goal_x is None or self.goal_y is None:
            return False

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.hypot(dx, dy)

        if distance <= max(self.arrive_distance, self.near_goal_distance):
            return False

        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
        yaw_error_deg = abs(math.degrees(yaw_error))

        return yaw_error_deg <= self.blind_yaw_limit_deg

    def enter_search_mode(self, now):
        if self.recovery_mode != 'SEARCH':
            self.reset_motion_state()

            self.recovery_mode = 'SEARCH'
            self.search_phase = 'STOP'

            self.search_phase_start_time = now
            self.search_start_time = now
            self.search_pause_start_time = None

            self.search_step_count = 0
            self.search_step_limit = 3
            self.accepted_pose_lost_start_time = None

            self.search_direction = (
                self.select_initial_search_direction()
            )

            self.publish_cmd('s')

            self.get_logger().warn(
                f'ENTER SEARCH MODE: '
                f'initial_direction={self.search_direction}, '
                f'last_marker_id={self.last_marker_id}, '
                f'last_offset_x={self.last_marker_offset_x:.1f}'
            )

    def enter_reacquire_mode(self, now):
        self.reset_motion_state()

        self.recovery_mode = 'REACQUIRE'
        self.reacquire_start_time = now

        self.search_phase = 'STOP'
        self.search_phase_start_time = None

        self.publish_cmd('s')

        self.get_logger().info(
            f'Marker reacquired '
            f'{self.reacquire_count}/'
            f'{self.reacquire_required_count}. '
            f'Hold stop before resuming.'
        )

    def handle_reacquire_mode(self, now):
        self.publish_cmd('s')

        if not self.is_control_pose_usable():
            self.enter_search_mode(now)
            return

        if self.accepted_pose_streak < self.reacquire_required_count:
            return

        if self.reacquire_start_time is None:
            self.reacquire_start_time = now
            return

        elapsed = (
            now - self.reacquire_start_time
        ).nanoseconds / 1e9

        if elapsed < self.reacquire_stop_sec:
            return

        self.recovery_mode = 'NORMAL'

        self.reset_motion_state()
        self.reset_search_state()

        self.get_logger().info(
            'REACQUIRE COMPLETE. Resume waypoint driving.'
        )

    def handle_search_mode(self, now):
        if (
            self.is_control_pose_usable()
            and self.accepted_pose_streak >= self.reacquire_required_count
        ):
            self.enter_reacquire_mode(now)
            return

        if self.search_start_time is None:
            self.search_start_time = now

        total_elapsed = (
            now - self.search_start_time
        ).nanoseconds / 1e9

        if total_elapsed >= self.search_max_active_sec:
            self.recovery_mode = 'SEARCH_PAUSE'
            self.search_pause_start_time = now

            self.publish_cmd('s')

            self.get_logger().warn(
                'Search active time exceeded. '
                'Pause briefly before another scan.'
            )
            return

        if self.search_phase_start_time is None:
            self.search_phase_start_time = now

        phase_elapsed = (
            now - self.search_phase_start_time
        ).nanoseconds / 1e9

        if self.search_phase == 'STOP':
            self.publish_cmd('s')

            if phase_elapsed >= self.search_stop_sec:
                self.search_phase = 'PIVOT'
                self.search_phase_start_time = now

                self.publish_cmd(
                    self.search_direction
                )

                self.get_logger().info(
                    f'SEARCH_PIVOT '
                    f'direction={self.search_direction}, '
                    f'step={self.search_step_count + 1}/'
                    f'{self.search_step_limit}'
                )

            return

        if self.search_phase == 'PIVOT':
            self.publish_cmd(
                self.search_direction
            )

            if phase_elapsed >= self.search_pivot_sec:
                self.publish_cmd('s')

                self.search_step_count += 1
                self.search_phase = 'STOP'
                self.search_phase_start_time = now

                if (
                    self.search_step_count
                    >= self.search_step_limit
                ):
                    self.reverse_search_direction()

            return

        self.search_phase = 'STOP'
        self.search_phase_start_time = now
        self.publish_cmd('s')

    def handle_search_pause(self, now):
        self.publish_cmd('s')

        if (
            self.is_control_pose_usable()
            and self.accepted_pose_streak >= self.reacquire_required_count
        ):
            self.enter_reacquire_mode(now)
            return

        if self.search_pause_start_time is None:
            self.search_pause_start_time = now
            return

        elapsed = (
            now - self.search_pause_start_time
        ).nanoseconds / 1e9

        if elapsed < self.search_pause_sec:
            return

        self.recovery_mode = 'SEARCH'
        self.search_start_time = now
        self.search_phase = 'STOP'
        self.search_phase_start_time = now

        self.search_step_count = 0
        self.search_step_limit = 3

        self.reverse_search_direction(
            increase_sweep=False
        )

        self.get_logger().warn(
            'SEARCH RESUMED after pause.'
        )

    def reverse_search_direction(
        self,
        increase_sweep=True
    ):
        if self.search_direction == 'q':
            self.search_direction = 'e'
        else:
            self.search_direction = 'q'

        self.search_step_count = 0

        if increase_sweep:
            self.search_step_limit = min(
                self.search_step_limit + 2,
                9
            )

        self.get_logger().info(
            f'SEARCH_DIRECTION_CHANGED: '
            f'direction={self.search_direction}, '
            f'next_step_limit={self.search_step_limit}'
        )

    def select_initial_search_direction(self):
        if self.last_marker_offset_x < -10.0:
            return 'q'

        if self.last_marker_offset_x > 10.0:
            return 'e'

        return 'q'


    def reset_target_tracking(self):
        self.current_target_min_distance = None
        self.current_target_last_distance = None

    def update_target_tracking(self, distance):
        if self.current_target_min_distance is None:
            self.current_target_min_distance = distance
        else:
            self.current_target_min_distance = min(
                self.current_target_min_distance,
                distance
            )

        self.current_target_last_distance = distance

    def skip_current_path_target(self, distance, yaw_error_deg, reason):
        if not self.using_path:
            return False

        if len(self.path_points) == 0:
            return False

        last_index = len(self.path_points) - 1

        if self.path_index >= last_index:
            return False

        old_index = self.path_index
        old_min_distance = self.current_target_min_distance

        self.path_index += 1
        self.select_path_target()
        self.reset_target_tracking()
        self.reset_motion_state()

        self.get_logger().warn(
            f'PATH WAYPOINT SKIPPED: '
            f'{old_index} -> {self.path_index}/{last_index}, '
            f'reason={reason}, '
            f'dist={distance:.3f}, '
            f'min_dist={old_min_distance}, '
            f'yaw_error={yaw_error_deg:.1f}deg, '
            f'next_goal=({self.goal_x:.3f},{self.goal_y:.3f})'
        )

        return True

    def should_skip_current_path_target(self, distance, yaw_error_deg):
        if not self.using_path:
            return False, ''

        if len(self.path_points) == 0:
            return False, ''

        last_index = len(self.path_points) - 1

        if self.path_index >= last_index:
            return False, ''

        min_dist = self.current_target_min_distance

        if distance <= self.intermediate_waypoint_reach_m:
            return True, 'intermediate_reach_radius'

        if min_dist is not None:
            if (
                min_dist <= self.waypoint_skip_min_dist_m
                and distance >= min_dist + self.waypoint_skip_increase_m
            ):
                return True, 'passed_and_distance_increased'

            if (
                min_dist <= 0.50
                and distance <= 0.65
                and abs(yaw_error_deg) >= self.waypoint_skip_yaw_deg
            ):
                return True, 'target_behind_after_miss'

        return False, ''

    def pivot_cmd_for_yaw(self, yaw_error_deg):
        if yaw_error_deg > 0.0:
            return 'q'
        return 'e'

    def get_variable_pivot_pulse_sec(self, yaw_error_deg):
        error = abs(yaw_error_deg)

        if error < 45.0:
            return 0.0

        if error < 60.0:
            return 0.10

        if error < 80.0:
            return 0.14

        if error < 110.0:
            return 0.19

        return 0.24

    def select_path_target(self):
        if not self.using_path or len(self.path_points) == 0:
            return

        if self.current_x is None or self.current_y is None:
            self.goal_x, self.goal_y = self.path_points[self.path_index]
            return

        last_index = len(self.path_points) - 1

        while self.path_index < last_index:
            tx, ty = self.path_points[self.path_index]
            dist = math.hypot(
                tx - self.current_x,
                ty - self.current_y
            )

            if dist >= self.path_target_lookahead_m:
                break

            self.path_index += 1

        self.goal_x, self.goal_y = self.path_points[self.path_index]

    def advance_path_target_if_needed(self, distance):
        if not self.using_path:
            return False

        if len(self.path_points) == 0:
            return False

        last_index = len(self.path_points) - 1

        if self.path_index >= last_index:
            return False

        if distance > self.path_waypoint_reach_m:
            return False

        old_index = self.path_index
        self.path_index += 1
        self.select_path_target()
        self.reset_target_tracking()

        self.reset_motion_state()

        self.get_logger().info(
            f'PATH WAYPOINT PASSED: '
            f'{old_index} -> {self.path_index}/'
            f'{last_index}, '
            f'next_goal=({self.goal_x:.3f},'
            f'{self.goal_y:.3f}), '
            f'dist={distance:.3f}'
        )

        return True

    def is_final_path_target(self):
        if not self.using_path:
            return True

        if len(self.path_points) == 0:
            return True

        return self.path_index >= len(self.path_points) - 1

    def run_waypoint_control(self, now):
        if not self.is_control_pose_usable():
            self.reset_motion_state()

            if self.marker_seen == 1 or self.is_marker_currently_detected():
                self.recovery_mode = 'WAITING_ACCEPTED_POSE'

                if self.should_continue_when_pose_unstable(now):
                    self.publish_cmd('w')
                    self.log_warning_throttled(
                        'Waypoint control: accepted pose unstable, but marker/path remain. '
                        'Continue no-stuck forward instead of stop.'
                    )
                    return

                if self.can_use_blind_grace(now):
                    self.publish_cmd('w')
                    self.log_warning_throttled(
                        'Waypoint control: accepted pose briefly unstable. '
                        'Use forward grace instead of immediate stop.'
                    )
                    return
            else:
                self.recovery_mode = 'WAITING_FOR_MARKER'

            self.publish_cmd('s')
            self.log_warning_throttled(
                'Waypoint control blocked: accepted pose not usable. '
                'Wait before search.'
            )
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.hypot(
            dx,
            dy
        )

        if self.advance_path_target_if_needed(distance):
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.hypot(
            dx,
            dy
        )

        target_yaw = math.atan2(
            dy,
            dx
        )

        yaw_error = self.normalize_angle(
            target_yaw - self.current_yaw
        )

        yaw_error_deg = math.degrees(
            yaw_error
        )

        self.update_target_tracking(distance)

        should_skip, skip_reason = self.should_skip_current_path_target(
            distance,
            yaw_error_deg
        )

        if should_skip:
            self.skip_current_path_target(
                distance,
                yaw_error_deg,
                skip_reason
            )
            return

        if (
            self.is_force_forward_active(now)
            and distance > self.arrive_distance
            and abs(yaw_error_deg) <= 45.0
        ):
            self.publish_cmd('w')
            self.log_status(
                distance,
                target_yaw,
                yaw_error_deg,
                'w'
            )
            return

        if distance <= self.arrive_distance:
            if self.using_path and not self.is_final_path_target():
                old_index = self.path_index
                self.path_index += 1
                self.select_path_target()
                self.reset_target_tracking()
                self.reset_motion_state()

                self.get_logger().info(
                    f'PATH WAYPOINT REACHED: '
                    f'{old_index} -> {self.path_index}/'
                    f'{len(self.path_points) - 1}, '
                    f'next_goal=({self.goal_x:.3f},'
                    f'{self.goal_y:.3f}), '
                    f'dist={distance:.3f}'
                )
                return

            self.arrived = True

            self.reset_motion_state()
            self.publish_cmd('s')

            if self.using_path:
                self.get_logger().info(
                    f'PATH ARRIVED: '
                    f'x={self.current_x:.3f}, '
                    f'y={self.current_y:.3f}, '
                    f'final_goal=({self.goal_x:.3f},'
                    f'{self.goal_y:.3f}), '
                    f'dist={distance:.3f}, '
                    f'path_count={len(self.path_points)}'
                )
            else:
                self.get_logger().info(
                    f'ARRIVED: '
                    f'x={self.current_x:.3f}, '
                    f'y={self.current_y:.3f}, '
                    f'goal=({self.goal_x:.3f},'
                    f'{self.goal_y:.3f}), '
                    f'dist={distance:.3f}'
                )
            return

        if distance <= self.near_goal_distance:
            if abs(yaw_error_deg) <= self.near_goal_yaw_forward_limit_deg:
                self.publish_cmd('w')
                self.log_status(
                    distance,
                    target_yaw,
                    yaw_error_deg,
                    'w'
                )
                return

            if self.using_path and not self.is_final_path_target():
                if abs(yaw_error_deg) >= self.waypoint_skip_yaw_deg:
                    if self.skip_current_path_target(distance, yaw_error_deg, 'near_target_large_yaw'):
                        return

            self.start_pivot(
                self.pivot_cmd_for_yaw(yaw_error_deg),
                now,
                distance,
                target_yaw,
                yaw_error_deg
            )
            return

        if (
            abs(yaw_error_deg)
            > self.hard_stop_angle_deg
        ):
            if yaw_error_deg > 0:
                self.start_pivot(
                    'q',
                    now,
                    distance,
                    target_yaw,
                    yaw_error_deg
                )
            else:
                self.start_pivot(
                    'e',
                    now,
                    distance,
                    target_yaw,
                    yaw_error_deg
                )
            return

        if distance <= 0.80 and abs(yaw_error_deg) <= self.near_goal_yaw_forward_limit_deg:
            self.publish_cmd('w')
            self.log_status(
                distance,
                target_yaw,
                yaw_error_deg,
                'w'
            )
            return

        if (
            abs(yaw_error_deg)
            > self.yaw_tolerance_deg
        ):
            self.start_pivot(
                self.pivot_cmd_for_yaw(yaw_error_deg),
                now,
                distance,
                target_yaw,
                yaw_error_deg
            )
            return

        self.publish_cmd('w')

        self.log_status(
            distance,
            target_yaw,
            yaw_error_deg,
            'w'
        )

    def run_blind_short_control(
        self,
        now,
        distance,
        target_yaw,
        yaw_error_deg
    ):
        if abs(yaw_error_deg) > self.yaw_tolerance_deg:
            self.publish_cmd('s')

            self.log_status(
                distance,
                target_yaw,
                yaw_error_deg,
                's'
            )
            return

        if distance <= self.arrive_distance:
            self.publish_cmd('s')
            return

        self.publish_cmd('w')

        self.log_status(
            distance,
            target_yaw,
            yaw_error_deg,
            'w'
        )

    def start_pivot(
        self,
        cmd,
        now,
        distance,
        target_yaw,
        yaw_error_deg
    ):
        self.update_pivot_escape_counter()

        if self.should_escape_pivot_loop(distance, yaw_error_deg):
            self.force_forward_start_time = now
            self.reset_motion_state()
            self.publish_cmd('w')
            self.get_logger().warn(
                f'PIVOT LOOP ESCAPE: force forward. '
                f'path_index={self.path_index}, '
                f'pivot_count={self.pivot_escape_count}, '
                f'dist={distance:.3f}, '
                f'yaw_error={yaw_error_deg:.1f}deg'
            )
            return

        self.mode = 'PIVOTING'
        self.pivot_cmd = cmd
        self.pivot_start_time = now
        self.pivot_stop_time = None
        self.current_pivot_pulse_sec = self.get_variable_pivot_pulse_sec(yaw_error_deg)

        if self.current_pivot_pulse_sec <= 0.0:
            self.reset_motion_state()
            self.publish_cmd('w')
            self.log_status(
                distance,
                target_yaw,
                yaw_error_deg,
                'w'
            )
            return

        self.publish_cmd(cmd)

        self.get_logger().info(
            f'START_PIVOT '
            f'cmd={cmd}, '
            f'pose=({self.current_x:.3f},'
            f'{self.current_y:.3f}), '
            f'yaw={math.degrees(self.current_yaw):.1f}deg, '
            f'goal=({self.goal_x:.3f},'
            f'{self.goal_y:.3f}), '
            f'dist={distance:.3f}, '
            f'target_yaw='
            f'{math.degrees(target_yaw):.1f}deg, '
            f'yaw_error={yaw_error_deg:.1f}deg, '
            f'pulse={self.current_pivot_pulse_sec:.2f}s, '
            f'source={self.source}, '
            f'marker_seen={self.marker_seen}, '
            f'recovery_mode={self.recovery_mode}'
        )

    def handle_pivoting(self, now):
        if self.pivot_start_time is None:
            self.reset_motion_state()
            self.publish_cmd('s')
            return

        if not self.is_control_pose_usable():
            self.reset_motion_state()
            self.handle_marker_lost(now)
            return

        elapsed = (
            now - self.pivot_start_time
        ).nanoseconds / 1e9

        active_pulse_sec = self.current_pivot_pulse_sec

        if elapsed >= active_pulse_sec:
            self.publish_cmd('s')

            self.pivot_stop_time = now
            self.mode = 'AFTER_PIVOT_STOP'

            self.get_logger().info(
                f'PIVOT_STOP after {elapsed:.2f}s / target {active_pulse_sec:.2f}s'
            )
            return

        self.publish_cmd(
            self.pivot_cmd
        )

    def handle_after_pivot_stop(self, now):
        if self.pivot_stop_time is None:
            self.reset_motion_state()
            self.publish_cmd('s')
            return

        elapsed = (
            now - self.pivot_stop_time
        ).nanoseconds / 1e9

        self.publish_cmd('s')

        if elapsed >= self.after_pivot_stop_sec:
            self.reset_motion_state()

            self.get_logger().info(
                'PIVOT_COOLDOWN_DONE. Recheck pose.'
            )

    def is_pose_fresh(self, now):
        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
            or self.last_pose_time is None
        ):
            return False

        elapsed = (
            now - self.last_pose_time
        ).nanoseconds / 1e9

        return elapsed <= self.pose_timeout_sec

    def is_marker_currently_detected(self):
        if self.last_aruco_time is None:
            return False

        now = self.get_clock().now()

        elapsed = (
            now - self.last_aruco_time
        ).nanoseconds / 1e9

        if elapsed > self.aruco_timeout_sec:
            return False

        return self.raw_marker_detected

    def is_control_pose_usable(self):
        if self.marker_seen != 1:
            return False

        if self.aruco_accepted != 1:
            return False

        return True

    def should_continue_when_pose_unstable(self, now):
        if self.goal_x is None or self.goal_y is None:
            return False

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        # Direct PoseStamped goal is valid even when no A* path exists.
        # If a path exists, this also works as path-following grace.
        marker_still_visible = (
            self.marker_seen == 1
            or self.is_marker_currently_detected()
        )

        if not marker_still_visible:
            return False

        if self.last_accepted_pose_time is None:
            return False

        elapsed = (
            now - self.last_accepted_pose_time
        ).nanoseconds / 1e9

        if elapsed > self.accepted_pose_wait_sec:
            return False

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)

        if distance <= self.arrive_distance:
            return False

        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
        yaw_error_deg = abs(math.degrees(yaw_error))

        if distance <= 0.80:
            return yaw_error_deg <= self.unstable_forward_yaw_limit_deg

        return yaw_error_deg <= self.blind_yaw_limit_deg

    def update_pivot_escape_counter(self):
        current_index = self.path_index if self.using_path else -1

        if current_index != self.pivot_escape_path_index:
            self.pivot_escape_path_index = current_index
            self.pivot_escape_count = 0

        self.pivot_escape_count += 1

    def should_escape_pivot_loop(self, distance, yaw_error_deg):
        if self.pivot_escape_count < self.pivot_escape_limit:
            return False

        if abs(yaw_error_deg) <= 25.0 and distance > self.arrive_distance and distance <= 0.50:
            return True

        return False

    def is_force_forward_active(self, now):
        if self.force_forward_start_time is None:
            return False

        elapsed = (
            now - self.force_forward_start_time
        ).nanoseconds / 1e9

        if elapsed <= self.pivot_escape_forward_sec:
            return True

        self.force_forward_start_time = None
        self.pivot_escape_count = 0
        return False


    def log_status(
        self,
        distance,
        target_yaw,
        yaw_error_deg,
        cmd
    ):
        now = self.get_clock().now()

        elapsed = (
            now - self.last_log_time
        ).nanoseconds / 1e9

        if elapsed < 0.25:
            return

        self.last_log_time = now

        self.get_logger().info(
            f'pose=({self.current_x:.3f},'
            f'{self.current_y:.3f}), '
            f'yaw={math.degrees(self.current_yaw):.1f}deg, '
            f'goal=({self.goal_x:.3f},'
            f'{self.goal_y:.3f}), '
            f'dist={distance:.3f}, '
            f'target_yaw={math.degrees(target_yaw):.1f}deg, '
            f'yaw_error={yaw_error_deg:.1f}deg, '
            f'cmd={cmd}, '
            f'mode={self.mode}, '
            f'recovery_mode={self.recovery_mode}, '
            f'source={self.source}, '
            f'marker_seen={self.marker_seen}, '
            f'aruco_accepted={self.aruco_accepted}, '
            f'raw_marker={1 if self.raw_marker_detected else 0}, '
            f'accepted_streak={self.accepted_pose_streak}, '
            f'reacquire_count={self.reacquire_count}, '
            f'using_path={1 if self.using_path else 0}, '
            f'path_index={self.path_index}/'
            f'{max(len(self.path_points) - 1, 0)}'
        )

    def log_warning_throttled(self, message):
        now = self.get_clock().now()

        elapsed = (
            now - self.last_warning_time
        ).nanoseconds / 1e9

        if elapsed < 1.0:
            return

        self.last_warning_time = now
        self.get_logger().warn(message)

    def publish_cmd(self, cmd):
        if cmd not in ['w', 's', 'q', 'e']:
            cmd = 's'

        msg = String()
        msg.data = cmd

        self.cmd_pub.publish(msg)
        self.last_cmd = cmd

    def parse_key_value_message(
        self,
        data,
        prefix
    ):
        if not data.startswith(prefix + ','):
            return None

        result = {}
        parts = data.split(',')

        for part in parts[1:]:
            if '=' not in part:
                continue

            key, value = part.split(
                '=',
                1
            )

            result[key.strip()] = value.strip()

        return result

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


def main(args=None):
    rclpy.init(args=args)

    node = WaypointDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_cmd('s')
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
