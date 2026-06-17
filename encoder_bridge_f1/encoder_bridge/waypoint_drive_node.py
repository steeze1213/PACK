import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WaypointDriveNode(Node):
    def __init__(self):
        super().__init__('waypoint_drive_node')

        self.pose_sub = self.create_subscription(
            String,
            '/relative_pose',
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            String,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.aruco_sub = self.create_subscription(
            String,
            '/aruco_marker',
            self.aruco_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/robot_cmd',
            10
        )

        self.goal_x = None
        self.goal_y = None

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.marker_seen = 0
        self.raw_marker_detected = False
        self.source = 'none'

        self.last_marker_id = -1
        self.last_marker_offset_x = 0.0

        self.last_pose_time = None
        self.last_aruco_time = None
        self.marker_lost_start_time = None

        self.pose_timeout_sec = 0.60
        self.aruco_timeout_sec = 0.40
        self.blind_grace_sec = 0.35

        self.reacquire_required_count = 3
        self.reacquire_count = 0
        self.reacquire_stop_sec = 0.20
        self.reacquire_start_time = None

        self.arrive_distance = 0.18
        self.near_goal_distance = 0.35

        self.yaw_tolerance_deg = 18.0
        self.hard_stop_angle_deg = 170.0

        self.pivot_pulse_sec = 0.30
        self.after_pivot_stop_sec = 0.15

        self.search_pivot_sec = 0.18
        self.search_stop_sec = 0.18
        self.search_pause_sec = 0.80
        self.search_max_active_sec = 8.0

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

        self.get_logger().info('waypoint_drive_node started.')
        self.get_logger().info(
            'Mode: waypoint drive + short blind driving + marker search recovery'
        )
        self.get_logger().info(
            'Subscribing: /relative_pose, /goal_pose, /aruco_marker'
        )
        self.get_logger().info('Publishing: /robot_cmd')
        self.get_logger().info(
            f'arrive_distance={self.arrive_distance}, '
            f'near_goal_distance={self.near_goal_distance}, '
            f'yaw_tolerance_deg={self.yaw_tolerance_deg}, '
            f'blind_grace_sec={self.blind_grace_sec}, '
            f'reacquire_required_count={self.reacquire_required_count}, '
            f'search_pivot_sec={self.search_pivot_sec}, '
            f'search_stop_sec={self.search_stop_sec}'
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
            self.source = str(
                parsed.get('source', 'unknown')
            )
        except (TypeError, ValueError):
            return

        self.last_pose_time = self.get_clock().now()

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

    def goal_callback(self, msg):
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

            self.arrived = False
            self.no_goal_logged = False

            self.reset_motion_state()
            self.reset_search_state()

            if self.is_marker_currently_detected():
                self.recovery_mode = 'NORMAL'
            else:
                self.recovery_mode = 'WAITING_FOR_MARKER'

            self.publish_cmd('s')

            self.get_logger().info(
                f'New goal received: '
                f'x={self.goal_x:.3f}, '
                f'y={self.goal_y:.3f}'
            )

        except (TypeError, ValueError):
            self.get_logger().warn(
                f'Invalid goal message: {data}'
            )

    def clear_goal(self):
        self.goal_x = None
        self.goal_y = None
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

    def reset_search_state(self):
        self.search_phase = 'STOP'
        self.search_phase_start_time = None
        self.search_start_time = None
        self.search_pause_start_time = None

        self.search_step_count = 0
        self.search_step_limit = 3

        self.reacquire_start_time = None
        self.reacquire_count = 0

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

        if not self.is_pose_fresh(now):
            self.reset_motion_state()
            self.publish_cmd('s')
            self.log_warning_throttled(
                'Relative pose timeout. Stop for safety.'
            )
            return

        marker_detected = self.is_marker_currently_detected()

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

        if self.recovery_mode == 'WAITING_FOR_MARKER':
            self.enter_search_mode(now)
            return

        if self.mode == 'PIVOTING':
            self.handle_pivoting(now)
            return

        if self.mode == 'AFTER_PIVOT_STOP':
            self.handle_after_pivot_stop(now)
            return

        self.run_waypoint_control(now)

    def handle_marker_detected(self, now):
        if self.recovery_mode in [
            'SEARCH',
            'SEARCH_PAUSE',
            'WAITING_FOR_MARKER'
        ]:
            if (
                self.reacquire_count
                >= self.reacquire_required_count
            ):
                self.enter_reacquire_mode(now)

            return

        if self.recovery_mode == 'REACQUIRE':
            return

        self.recovery_mode = 'NORMAL'

    def handle_marker_lost(self, now):
        if self.marker_lost_start_time is None:
            self.marker_lost_start_time = now

        lost_duration = (
            now - self.marker_lost_start_time
        ).nanoseconds / 1e9

        if self.recovery_mode == 'REACQUIRE':
            self.get_logger().warn(
                'Marker lost during reacquire. Return to search.'
            )
            self.enter_search_mode(now)
            return

        if self.recovery_mode in [
            'SEARCH',
            'SEARCH_PAUSE'
        ]:
            return

        if lost_duration <= self.blind_grace_sec:
            if self.recovery_mode != 'BLIND_SHORT':
                self.recovery_mode = 'BLIND_SHORT'

                self.get_logger().warn(
                    f'Marker temporarily lost. '
                    f'Continue with pose for up to '
                    f'{self.blind_grace_sec:.2f}s.'
                )

            return

        self.enter_search_mode(now)

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

        if not self.is_marker_currently_detected():
            self.enter_search_mode(now)
            return

        if (
            self.reacquire_count
            < self.reacquire_required_count
        ):
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
            self.is_marker_currently_detected()
            and self.reacquire_count
            >= self.reacquire_required_count
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
            self.is_marker_currently_detected()
            and self.reacquire_count
            >= self.reacquire_required_count
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

    def run_waypoint_control(self, now):
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

        if distance <= self.arrive_distance:
            self.arrived = True

            self.reset_motion_state()
            self.publish_cmd('s')

            self.get_logger().info(
                f'ARRIVED: '
                f'x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}, '
                f'goal=({self.goal_x:.3f},'
                f'{self.goal_y:.3f}), '
                f'dist={distance:.3f}'
            )
            return

        if self.recovery_mode == 'BLIND_SHORT':
            self.run_blind_short_control(
                now,
                distance,
                target_yaw,
                yaw_error_deg
            )
            return

        if distance <= self.near_goal_distance:
            if abs(yaw_error_deg) <= 90.0:
                cmd = 'w'
            else:
                cmd = 's'

            self.publish_cmd(cmd)

            self.log_status(
                distance,
                target_yaw,
                yaw_error_deg,
                cmd
            )
            return

        if (
            abs(yaw_error_deg)
            > self.hard_stop_angle_deg
        ):
            self.publish_cmd('s')

            self.log_status(
                distance,
                target_yaw,
                yaw_error_deg,
                's'
            )
            return

        if (
            abs(yaw_error_deg)
            > self.yaw_tolerance_deg
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
        self.mode = 'PIVOTING'
        self.pivot_cmd = cmd
        self.pivot_start_time = now
        self.pivot_stop_time = None

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
            f'source={self.source}, '
            f'marker_seen={self.marker_seen}, '
            f'recovery_mode={self.recovery_mode}'
        )

    def handle_pivoting(self, now):
        if self.pivot_start_time is None:
            self.reset_motion_state()
            self.publish_cmd('s')
            return

        if not self.is_marker_currently_detected():
            self.reset_motion_state()
            self.handle_marker_lost(now)

            if self.recovery_mode == 'SEARCH':
                self.handle_search_mode(now)
            else:
                self.publish_cmd('s')

            return

        elapsed = (
            now - self.pivot_start_time
        ).nanoseconds / 1e9

        if elapsed >= self.pivot_pulse_sec:
            self.publish_cmd('s')

            self.pivot_stop_time = now
            self.mode = 'AFTER_PIVOT_STOP'

            self.get_logger().info(
                f'PIVOT_STOP after {elapsed:.2f}s'
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
            f'raw_marker={1 if self.raw_marker_detected else 0}, '
            f'reacquire_count={self.reacquire_count}'
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
        node.publish_cmd('s')

    node.publish_cmd('s')
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
