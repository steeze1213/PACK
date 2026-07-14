#!/usr/bin/env python3
# flake8: noqa: E501

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


class F1ToF2FollowNode(Node):
    def __init__(self):
        super().__init__('f1_to_f2_follow_node')

        self.follower_pose_sub = self.create_subscription(
            String,
            '/relative_pose',
            self.follower_pose_callback,
            10
        )

        self.leader_pose_sub = self.create_subscription(
            String,
            '/f1/relative_pose',
            self.leader_pose_callback,
            10
        )

        self.tb3_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.tb3_pose_callback,
            10
        )

        self.target_marker_sub = self.create_subscription(
            String,
            '/target_marker',
            self.target_marker_callback,
            10
        )

        self.target_marker_cmd_sub = self.create_subscription(
            String,
            '/target_marker_cmd',
            self.target_marker_cmd_callback,
            10
        )

        self.follow_enable_sub = self.create_subscription(
            String,
            '/follow_enable',
            self.follow_enable_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/robot_cmd',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/follow_status',
            10
        )

        self.follow_enabled = False
        self.active_leader_name = 'F1'

        self.follower_x = None
        self.follower_y = None
        self.follower_yaw = None
        self.follower_pose_source = 'none'

        self.leader_x = None
        self.leader_y = None
        self.leader_yaw = None
        self.leader_pose_source = 'none'

        self.previous_leader_x = None
        self.previous_leader_y = None
        self.previous_leader_yaw = None
        self.previous_leader_pose_time = None

        self.leader_linear_speed = 0.0
        self.leader_angular_speed = 0.0

        self.target_detected = False
        self.target_rel_x = 0.0
        self.target_rel_y = 0.0
        self.target_rel_z = 0.0
        self.target_distance = 0.0
        self.target_bearing_deg = 0.0
        self.target_offset_x = 0.0

        self.last_follower_pose_time = None
        self.last_leader_pose_time = None
        self.last_target_marker_time = None

        self.declare_follow_parameters()
        self.load_follow_parameters()

        if self.target_marker_id == 159:
            self.active_leader_name = 'TB3'
        else:
            self.active_leader_name = 'F1'

        self.motion_state = 'IDLE'
        self.active_cmd = 's'

        self.motion_start_time = None
        self.motion_duration_sec = 0.0
        self.stop_start_time = None

        self.last_cmd = 's'
        self.follow_mode = 'DISABLED'
        self.last_control_source = 'NONE'
        self.transition_stop_until = None
        self.marker_pose_mismatch_count = 0
        self.follower_ahead_count = 0

        self.last_log_time = self.get_clock().now()
        self.last_status_time = self.get_clock().now()
        self.last_warning_time = self.get_clock().now()

        self.control_interval = 0.05

        self.timer = self.create_timer(
            self.control_interval,
            self.control_loop
        )

        self.get_logger().info('F2 dynamic follow node started.')
        self.get_logger().info('Follower pose topic: /relative_pose')
        self.get_logger().info('F1 leader pose topic: /f1/relative_pose')
        self.get_logger().info('TB3 leader pose topic: /amcl_pose')
        self.get_logger().info('Target marker topic: /target_marker')
        self.get_logger().info('Target marker command topic: /target_marker_cmd')

        self.get_logger().info(
            f'Initial marker ID {self.target_marker_id}, active leader={self.active_leader_name}'
        )

        self.get_logger().info('158 = follow F1, 159 = follow TB3.')

        self.get_logger().info(
            f'Follow distance={self.follow_distance:.2f}m, '
            f'map_follow_center_distance={self.map_follow_center_distance:.2f}m'
        )

        self.get_logger().info(
            f'Timeouts: follower_pose={self.follower_pose_timeout_sec:.2f}s, '
            f'leader_pose={self.leader_pose_timeout_sec:.2f}s, '
            f'target_marker={self.target_marker_timeout_sec:.2f}s'
        )

        self.get_logger().info(
            f'Pulses: marker_far_w={self.marker_far_forward_pulse_sec:.2f}s, '
            f'marker_near_w={self.marker_near_forward_pulse_sec:.2f}s, '
            f'marker_align={self.marker_align_pulse_sec:.2f}s, '
            f'map_w={self.map_forward_pulse_sec:.2f}s, '
            f'map_pivot={self.map_pivot_pulse_sec:.2f}s, '
            f'stop_pause={self.command_stop_pause_sec:.2f}s'
        )

    def follower_pose_callback(self, msg):
        parsed = self.parse_key_value_message(
            msg.data,
            'RELPOSE'
        )

        if parsed is None:
            return

        try:
            self.follower_x = float(parsed.get('x'))
            self.follower_y = float(parsed.get('y'))
            self.follower_yaw = float(parsed.get('yaw'))
            self.follower_pose_source = str(
                parsed.get('source', 'unknown')
            )

        except (TypeError, ValueError):
            return

        self.last_follower_pose_time = self.get_clock().now()

    def leader_pose_callback(self, msg):
        if self.active_leader_name != 'F1':
            return

        parsed = self.parse_key_value_message(
            msg.data,
            'RELPOSE'
        )

        if parsed is None:
            return

        try:
            new_x = float(parsed.get('x'))
            new_y = float(parsed.get('y'))
            new_yaw = float(parsed.get('yaw'))
            new_source = str(parsed.get('source', 'unknown'))

        except (TypeError, ValueError):
            return

        self.update_active_leader_pose(
            new_x,
            new_y,
            new_yaw,
            'F1_' + new_source
        )

    def tb3_pose_callback(self, msg):
        if self.active_leader_name != 'TB3':
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        new_x = float(position.x)
        new_y = float(position.y)
        new_yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        self.update_active_leader_pose(
            new_x,
            new_y,
            new_yaw,
            'TB3_AMCL'
        )

    def update_active_leader_pose(
        self,
        new_x,
        new_y,
        new_yaw,
        new_source
    ):
        now = self.get_clock().now()

        if (
            self.previous_leader_x is not None
            and self.previous_leader_y is not None
            and self.previous_leader_yaw is not None
            and self.previous_leader_pose_time is not None
        ):
            dt = (now - self.previous_leader_pose_time).nanoseconds / 1e9

            if dt >= 0.02:
                distance_delta = math.hypot(
                    new_x - self.previous_leader_x,
                    new_y - self.previous_leader_y
                )

                yaw_delta = self.normalize_angle(
                    new_yaw - self.previous_leader_yaw
                )

                calculated_linear_speed = distance_delta / dt
                calculated_angular_speed = yaw_delta / dt

                if calculated_linear_speed <= 2.0:
                    self.leader_linear_speed = calculated_linear_speed

                if abs(calculated_angular_speed) <= 10.0:
                    self.leader_angular_speed = calculated_angular_speed

        self.leader_x = new_x
        self.leader_y = new_y
        self.leader_yaw = new_yaw
        self.leader_pose_source = new_source

        self.previous_leader_x = new_x
        self.previous_leader_y = new_y
        self.previous_leader_yaw = new_yaw
        self.previous_leader_pose_time = now

        self.last_leader_pose_time = now

    def target_marker_callback(self, msg):
        parsed = self.parse_key_value_message(
            msg.data,
            'TARGET_MARKER'
        )

        if parsed is None:
            return

        now = self.get_clock().now()
        self.last_target_marker_time = now

        try:
            detected = int(parsed.get('detected', 0))

        except (TypeError, ValueError):
            detected = 0

        if detected != 1:
            self.reset_marker_measurement()
            self.target_detected = False
            return

        try:
            marker_id = int(parsed.get('id', -1))

            if marker_id != self.target_marker_id:
                self.target_detected = False
                return

            self.target_rel_x = float(parsed.get('rel_x', 0.0))
            self.target_rel_y = float(parsed.get('rel_y', 0.0))
            self.target_rel_z = float(parsed.get('rel_z', 0.0))

            raw_distance = float(parsed.get('planar_distance', 0.0))
            self.target_distance = self.filter_marker_distance(raw_distance)

            raw_bearing_deg = float(parsed.get('bearing_yaw_deg', 0.0))
            self.target_bearing_deg = self.filter_marker_bearing(raw_bearing_deg)

            self.target_offset_x = float(parsed.get('offset_x', 0.0))

            self.target_detected = self.target_distance > 0.0

        except (TypeError, ValueError):
            self.reset_marker_measurement()
            self.target_detected = False

    def target_marker_cmd_callback(self, msg):
        raw = msg.data.strip()

        try:
            marker_id = int(raw)
        except ValueError:
            self.get_logger().warn(
                f'Invalid target_marker_cmd ignored: {raw}'
            )
            return

        if marker_id not in [158, 159]:
            self.get_logger().warn(
                f'Unsupported F2 target marker ignored: {marker_id}'
            )
            return

        old_id = self.target_marker_id
        old_leader = self.active_leader_name

        self.target_marker_id = marker_id

        if marker_id == 158:
            self.active_leader_name = 'F1'
        else:
            self.active_leader_name = 'TB3'

        self.reset_marker_measurement()
        self.reset_leader_pose_state()
        self.marker_pose_mismatch_count = 0
        self.follower_ahead_count = 0
        self.last_control_source = 'NONE'

        self.reset_motion_state()
        self.publish_cmd('s')

        self.transition_stop_until = (
            self.get_clock().now()
            + Duration(seconds=self.mode_transition_stop_sec)
        )

        self.get_logger().warn(
            f'F2 follow target changed: {old_id}/{old_leader} -> '
            f'{self.target_marker_id}/{self.active_leader_name}'
        )

    def follow_enable_callback(self, msg):
        command = msg.data.strip().lower()

        if command in ['start', 'on', 'enable', '1']:
            self.follow_enabled = True
            self.follow_mode = 'WAITING'

            self.reset_motion_state()
            self.publish_cmd('s')

            self.get_logger().warn('F2 FOLLOW ENABLED')
            return

        if command in ['stop', 'off', 'disable', '0']:
            self.follow_enabled = False
            self.follow_mode = 'DISABLED'

            self.reset_motion_state()
            self.publish_cmd('s')

            self.get_logger().warn('F2 FOLLOW DISABLED')

    def control_loop(self):
        now = self.get_clock().now()

        if not self.follow_enabled:
            self.follow_mode = 'DISABLED'
            self.reset_motion_state()
            self.publish_cmd('s')
            self.publish_status(now)
            return

        if self.motion_state == 'COMMAND':
            if self.should_abort_active_command(now):
                self.reset_motion_state()
                self.publish_cmd('s')
                self.publish_status(now)
                return

            self.handle_active_command(now)
            self.publish_status(now)
            return

        if self.motion_state == 'STOP_PAUSE':
            self.handle_stop_pause(now)
            self.publish_status(now)
            return

        if self.transition_stop_until is not None:
            if now < self.transition_stop_until:
                self.follow_mode = 'MODE_TRANSITION_STOP'
                self.publish_cmd('s')
                self.publish_status(now)
                return

            self.transition_stop_until = None

        marker_available = self.is_target_marker_fresh(now)

        if not self.is_follower_pose_fresh(now):
            self.follow_mode = 'F2_POSE_LOST'
            self.publish_cmd('s')
            self.log_warning_throttled('F2 relative pose timeout. Stop.')
            self.publish_status(now)
            return

        if (
            marker_available
            and self.target_distance > 0.0
            and self.target_distance <= self.marker_use_max_distance
        ):
            if not self.is_marker_pose_consistent():
                self.follow_mode = 'MARKER_POSE_MISMATCH'
                self.publish_cmd('s')
                self.log_warning_throttled(
                    'Marker distance and map distance mismatch. Stop.'
                )
                self.publish_status(now)
                return

            if self.handle_control_source_change('MARKER', now):
                self.follow_mode = 'MODE_TRANSITION_STOP'
                self.publish_status(now)
                return

            self.run_marker_follow(now)
            self.publish_status(now)
            return

        if not self.is_leader_pose_fresh(now):
            if self.active_leader_name == 'TB3':
                self.follow_mode = 'TB3_POSE_LOST'
                warning = 'Marker lost and TB3 AMCL pose timeout. Stop.'
            else:
                self.follow_mode = 'F1_POSE_LOST'
                warning = 'Marker lost and F1 relative pose timeout. Stop.'

            self.publish_cmd('s')
            self.log_warning_throttled(warning)
            self.publish_status(now)
            return

        if self.handle_control_source_change('MAP', now):
            self.follow_mode = 'MODE_TRANSITION_STOP'
            self.publish_status(now)
            return

        self.run_map_follow(now)
        self.publish_status(now)

    def run_marker_follow(self, now):
        distance = self.target_distance
        bearing_deg = self.target_bearing_deg

        if distance <= self.marker_emergency_distance:
            self.follow_mode = 'EMERGENCY_TOO_CLOSE'
            self.publish_cmd('s')
            self.log_status(f'EMERGENCY distance={distance:.3f}m')
            return

        if distance <= self.marker_too_close_distance:
            self.follow_mode = 'TOO_CLOSE'
            self.publish_cmd('s')
            self.log_status(f'TOO_CLOSE distance={distance:.3f}m')
            return

        if abs(bearing_deg) >= self.marker_hard_stop_angle_deg:
            self.follow_mode = 'MARKER_HARD_ALIGN'

            if bearing_deg > 0.0:
                command = 'e'
            else:
                command = 'q'

            self.start_command(
                command,
                self.marker_align_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_HARD_ALIGN distance={distance:.3f} '
                f'bearing={bearing_deg:.1f} cmd={command}'
            )
            return

        if abs(bearing_deg) > self.marker_align_tolerance_deg:
            self.follow_mode = 'MARKER_ALIGN'

            if bearing_deg > 0.0:
                command = 'e'
            else:
                command = 'q'

            self.start_command(
                command,
                self.marker_align_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_ALIGN distance={distance:.3f} '
                f'bearing={bearing_deg:.1f} cmd={command}'
            )
            return

        if abs(bearing_deg) > self.marker_forward_limit_deg:
            self.follow_mode = 'MARKER_FORWARD_BLOCKED'
            self.publish_cmd('s')

            self.log_status(
                f'FORWARD_BLOCKED distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )
            return

        if distance >= self.marker_far_distance:
            self.follow_mode = 'MARKER_FORWARD_FAR'

            self.start_command(
                'w',
                self.marker_far_forward_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_FORWARD_FAR distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )
            return

        if distance >= self.marker_pulse_distance:
            self.follow_mode = 'MARKER_FORWARD_PULSE'

            self.start_command(
                'w',
                self.marker_near_forward_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_FORWARD_PULSE distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )
            return

        leader_moving = self.is_leader_moving(now)

        if distance > self.marker_hold_max_distance and leader_moving:
            self.follow_mode = 'MARKER_MOVING_PULSE'

            self.start_command(
                'w',
                self.marker_moving_forward_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_MOVING_PULSE distance={distance:.3f} '
                f'leader_speed={self.leader_linear_speed:.3f}'
            )
            return

        if self.marker_hold_min_distance <= distance <= self.marker_hold_max_distance:
            self.follow_mode = 'MARKER_HOLD'
            self.publish_cmd('s')

            self.log_status(
                f'MARKER_HOLD distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )
            return

        self.follow_mode = 'MARKER_STOP'
        self.publish_cmd('s')
        self.log_status(f'MARKER_STOP distance={distance:.3f}')

    def run_map_follow(self, now):
        leader_to_follower_along_heading = (
            (self.follower_x - self.leader_x) * math.cos(self.leader_yaw)
            + (self.follower_y - self.leader_y) * math.sin(self.leader_yaw)
        )

        lateral_error = (
            -(self.follower_x - self.leader_x) * math.sin(self.leader_yaw)
            + (self.follower_y - self.leader_y) * math.cos(self.leader_yaw)
        )

        if self.is_follower_ahead_of_leader(
            leader_to_follower_along_heading,
            lateral_error
        ):
            self.follow_mode = f'{self.active_leader_name}_FOLLOWER_NOT_BEHIND_LEADER'
            self.publish_cmd('s')
            self.log_status(
                f'FOLLOWER_NOT_BEHIND_LEADER '
                f'along={leader_to_follower_along_heading:.3f} '
                f'lateral={lateral_error:.3f}'
            )
            return

        follow_x = (
            self.leader_x
            - self.map_follow_center_distance * math.cos(self.leader_yaw)
        )

        follow_y = (
            self.leader_y
            - self.map_follow_center_distance * math.sin(self.leader_yaw)
        )

        dx = follow_x - self.follower_x
        dy = follow_y - self.follower_y

        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)

        yaw_error = self.normalize_angle(target_yaw - self.follower_yaw)
        yaw_error_deg = math.degrees(yaw_error)

        if distance <= self.map_arrive_distance:
            self.follow_mode = f'{self.active_leader_name}_MAP_HOLD'
            self.publish_cmd('s')

            self.log_status(
                f'MAP_HOLD distance={distance:.3f} '
                f'follow=({follow_x:.3f},{follow_y:.3f})'
            )
            return

        if abs(yaw_error_deg) >= self.map_hard_stop_angle_deg:
            self.follow_mode = f'{self.active_leader_name}_MAP_REVERSE_DIRECTION'
            self.publish_cmd('s')

            self.log_status(
                f'MAP_REVERSE_DIRECTION distance={distance:.3f} '
                f'yaw_error={yaw_error_deg:.1f}'
            )
            return

        if abs(yaw_error_deg) > self.map_yaw_tolerance_deg:
            self.follow_mode = f'{self.active_leader_name}_MAP_ALIGN'

            if yaw_error_deg > 0.0:
                command = 'q'
            else:
                command = 'e'

            self.start_command(
                command,
                self.map_pivot_pulse_sec,
                now
            )

            self.log_status(
                f'MAP_ALIGN distance={distance:.3f} '
                f'yaw_error={yaw_error_deg:.1f} cmd={command}'
            )
            return

        self.follow_mode = f'{self.active_leader_name}_MAP_FORWARD'

        self.start_command(
            'w',
            self.map_forward_pulse_sec,
            now
        )

        self.log_status(
            f'MAP_FORWARD distance={distance:.3f} '
            f'yaw_error={yaw_error_deg:.1f} '
            f'follow=({follow_x:.3f},{follow_y:.3f})'
        )

    def start_command(
        self,
        command,
        duration_sec,
        now
    ):
        if command not in ['w', 'q', 'e']:
            self.publish_cmd('s')
            return

        self.motion_state = 'COMMAND'
        self.active_cmd = command

        self.motion_start_time = now
        self.motion_duration_sec = duration_sec
        self.stop_start_time = None

        self.publish_cmd(command)

    def handle_active_command(self, now):
        if self.motion_start_time is None:
            self.reset_motion_state()
            self.publish_cmd('s')
            return

        elapsed = (now - self.motion_start_time).nanoseconds / 1e9

        if elapsed >= self.motion_duration_sec:
            self.publish_cmd('s')
            self.motion_state = 'STOP_PAUSE'
            self.stop_start_time = now
            return

        self.publish_cmd(self.active_cmd)

    def handle_stop_pause(self, now):
        self.publish_cmd('s')

        if self.stop_start_time is None:
            self.reset_motion_state()
            return

        elapsed = (now - self.stop_start_time).nanoseconds / 1e9

        if elapsed >= self.command_stop_pause_sec:
            self.reset_motion_state()

    def reset_motion_state(self):
        self.motion_state = 'IDLE'
        self.active_cmd = 's'

        self.motion_start_time = None
        self.motion_duration_sec = 0.0
        self.stop_start_time = None

    def reset_leader_pose_state(self):
        self.leader_x = None
        self.leader_y = None
        self.leader_yaw = None
        self.leader_pose_source = 'none'

        self.previous_leader_x = None
        self.previous_leader_y = None
        self.previous_leader_yaw = None
        self.previous_leader_pose_time = None

        self.leader_linear_speed = 0.0
        self.leader_angular_speed = 0.0
        self.last_leader_pose_time = None

    def is_follower_pose_fresh(self, now):
        if (
            self.follower_x is None
            or self.follower_y is None
            or self.follower_yaw is None
            or self.last_follower_pose_time is None
        ):
            return False

        elapsed = (now - self.last_follower_pose_time).nanoseconds / 1e9
        return elapsed <= self.follower_pose_timeout_sec

    def is_leader_pose_fresh(self, now):
        if (
            self.leader_x is None
            or self.leader_y is None
            or self.leader_yaw is None
            or self.last_leader_pose_time is None
        ):
            return False

        elapsed = (now - self.last_leader_pose_time).nanoseconds / 1e9
        return elapsed <= self.leader_pose_timeout_sec

    def is_target_marker_fresh(self, now):
        if not self.target_detected or self.last_target_marker_time is None:
            return False

        elapsed = (now - self.last_target_marker_time).nanoseconds / 1e9
        return elapsed <= self.target_marker_timeout_sec

    def is_marker_pose_consistent(self):
        now = self.get_clock().now()

        if (
            not self.is_follower_pose_fresh(now)
            or not self.is_leader_pose_fresh(now)
        ):
            return True

        map_distance = math.hypot(
            self.leader_x - self.follower_x,
            self.leader_y - self.follower_y
        )

        expected_map_distance = (
            self.target_distance
            + self.leader_pose_to_rear_marker_offset
            + self.follower_pose_to_front_camera_offset
        )

        if abs(map_distance - expected_map_distance) <= self.marker_pose_distance_max_error:
            self.marker_pose_mismatch_count = 0
            return True

        self.marker_pose_mismatch_count += 1

        return self.marker_pose_mismatch_count < self.marker_pose_mismatch_required_count

    def is_follower_ahead_of_leader(self, along_heading, lateral_error):
        if (
            along_heading <= self.follower_ahead_stop_distance
            or abs(lateral_error) > self.follower_ahead_lateral_tolerance
        ):
            self.follower_ahead_count = 0
            return False

        self.follower_ahead_count += 1

        return self.follower_ahead_count >= self.follower_ahead_required_count

    def should_abort_active_command(self, now):
        if self.last_control_source == 'MARKER':
            if not self.is_follower_pose_fresh(now):
                self.follow_mode = 'F2_POSE_LOST'
                return True

            if not self.is_target_marker_fresh(now):
                self.follow_mode = 'MARKER_LOST_DURING_COMMAND'
                return True

            if self.target_distance <= self.marker_too_close_distance:
                self.follow_mode = 'TOO_CLOSE'
                return True

        if self.last_control_source == 'MAP':
            if not self.is_follower_pose_fresh(now):
                self.follow_mode = 'F2_POSE_LOST'
                return True

            if not self.is_leader_pose_fresh(now):
                if self.active_leader_name == 'TB3':
                    self.follow_mode = 'TB3_POSE_LOST'
                else:
                    self.follow_mode = 'F1_POSE_LOST'
                return True

        return False

    def handle_control_source_change(self, source, now):
        if self.last_control_source in ['NONE', source]:
            self.last_control_source = source
            return False

        self.last_control_source = source
        self.reset_motion_state()
        self.publish_cmd('s')
        self.transition_stop_until = now + Duration(
            seconds=self.mode_transition_stop_sec
        )
        return True

    def is_leader_moving(self, now):
        if not self.is_leader_pose_fresh(now):
            return False

        return (
            abs(self.leader_linear_speed) >= self.leader_moving_speed
            or abs(self.leader_angular_speed) >= self.leader_turning_speed
        )

    def publish_cmd(self, command):
        if command not in ['w', 's', 'q', 'e']:
            command = 's'

        msg = String()
        msg.data = command

        try:
            if rclpy.ok():
                self.cmd_pub.publish(msg)
                self.last_cmd = command

        except Exception as error:
            self.get_logger().warn(
                f'publish command failed during shutdown: {error}'
            )

    def publish_status(self, now):
        elapsed = (now - self.last_status_time).nanoseconds / 1e9

        if elapsed < 0.25:
            return

        self.last_status_time = now

        marker_fresh = self.is_target_marker_fresh(now)
        follower_pose_fresh = self.is_follower_pose_fresh(now)
        leader_pose_fresh = self.is_leader_pose_fresh(now)

        status = (
            f'FOLLOW_STATUS,'
            f'enabled={1 if self.follow_enabled else 0},'
            f'mode={self.follow_mode},'
            f'active_leader={self.active_leader_name},'
            f'target_marker_id={self.target_marker_id},'
            f'cmd={self.last_cmd},'
            f'motion_state={self.motion_state},'
            f'follower_pose_fresh={1 if follower_pose_fresh else 0},'
            f'leader_pose_fresh={1 if leader_pose_fresh else 0},'
            f'target_detected={1 if marker_fresh else 0},'
            f'target_distance={self.target_distance:.3f},'
            f'target_bearing_deg={self.target_bearing_deg:.2f},'
            f'target_rel_x={self.target_rel_x:.3f},'
            f'target_rel_z={self.target_rel_z:.3f},'
            f'follower_x={self.value_or_zero(self.follower_x):.3f},'
            f'follower_y={self.value_or_zero(self.follower_y):.3f},'
            f'follower_yaw={self.value_or_zero(self.follower_yaw):.4f},'
            f'leader_x={self.value_or_zero(self.leader_x):.3f},'
            f'leader_y={self.value_or_zero(self.leader_y):.3f},'
            f'leader_yaw={self.value_or_zero(self.leader_yaw):.4f},'
            f'leader_linear={self.leader_linear_speed:.3f},'
            f'leader_angular={self.leader_angular_speed:.3f},'
            f'follower_source={self.follower_pose_source},'
            f'leader_source={self.leader_pose_source},'
            f'follower_timeout={self.follower_pose_timeout_sec:.2f},'
            f'leader_timeout={self.leader_pose_timeout_sec:.2f},'
            f'target_timeout={self.target_marker_timeout_sec:.2f},'
            f'marker_far_pulse={self.marker_far_forward_pulse_sec:.2f},'
            f'map_forward_pulse={self.map_forward_pulse_sec:.2f}'
        )

        msg = String()
        msg.data = status

        try:
            if rclpy.ok():
                self.status_pub.publish(msg)

        except Exception as error:
            self.get_logger().warn(
                f'publish status failed during shutdown: {error}'
            )

    def log_status(self, message):
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9

        if elapsed < 0.30:
            return

        self.last_log_time = now

        self.get_logger().info(
            f'{message}, mode={self.follow_mode}, '
            f'leader={self.active_leader_name}, '
            f'target={self.target_marker_id}, '
            f'cmd={self.last_cmd}, '
            f'follower_source={self.follower_pose_source}, '
            f'leader_source={self.leader_pose_source}'
        )

    def log_warning_throttled(self, message):
        now = self.get_clock().now()
        elapsed = (now - self.last_warning_time).nanoseconds / 1e9

        if elapsed < 1.0:
            return

        self.last_warning_time = now
        self.get_logger().warn(message)

    def parse_key_value_message(self, data, prefix):
        if not data.startswith(prefix + ','):
            return None

        result = {}
        parts = data.split(',')

        for part in parts[1:]:
            if '=' not in part:
                continue

            key, value = part.split('=', 1)
            result[key.strip()] = value.strip()

        return result

    def declare_follow_parameters(self):
        self.declare_parameter('target_marker_id', 158)

        # Follow distances.
        self.declare_parameter('follow_distance', 0.50)
        self.declare_parameter('map_follow_center_distance', 0.50)
        self.declare_parameter('leader_pose_to_rear_marker_offset', 0.0)
        self.declare_parameter('follower_pose_to_front_camera_offset', 0.0)
        self.declare_parameter('desired_bumper_gap', -1.0)
        self.declare_parameter('desired_camera_marker_distance', 0.0)

        # Marker distance gates.
        self.declare_parameter('marker_use_max_distance', 1.50)
        self.declare_parameter('marker_far_distance', 0.45)
        self.declare_parameter('marker_pulse_distance', 0.35)
        self.declare_parameter('marker_hold_min_distance', 0.27)
        self.declare_parameter('marker_hold_max_distance', 0.33)
        self.declare_parameter('marker_too_close_distance', 0.22)
        self.declare_parameter('marker_emergency_distance', 0.17)

        # Marker angle gates.
        self.declare_parameter('marker_align_tolerance_deg', 10.0)
        self.declare_parameter('marker_forward_limit_deg', 18.0)
        self.declare_parameter('marker_hard_stop_angle_deg', 25.0)

        # Map follow gates.
        self.declare_parameter('map_arrive_distance', 0.15)
        self.declare_parameter('map_yaw_tolerance_deg', 15.0)
        self.declare_parameter('map_hard_stop_angle_deg', 150.0)

        # Filters and timeouts.
        self.declare_parameter('marker_distance_filter_alpha', 0.35)
        self.declare_parameter('target_marker_timeout_sec', 1.00)
        self.declare_parameter('follower_pose_timeout_sec', 1.30)
        self.declare_parameter('leader_pose_timeout_sec', 1.50)

        # Pose consistency.
        self.declare_parameter('marker_pose_distance_max_error', 0.50)
        self.declare_parameter('marker_pose_mismatch_required_count', 3)
        self.declare_parameter('mode_transition_stop_sec', 0.15)

        # Follower position safety.
        self.declare_parameter('follower_ahead_stop_distance', 0.10)
        self.declare_parameter('follower_ahead_lateral_tolerance', 0.35)
        self.declare_parameter('follower_ahead_required_count', 3)

        # Leader moving detection.
        self.declare_parameter('leader_moving_speed', 0.015)
        self.declare_parameter('leader_turning_speed', 0.05)

        # Command pulse parameters.
        self.declare_parameter('marker_align_pulse_sec', 0.16)
        self.declare_parameter('marker_far_forward_pulse_sec', 0.30)
        self.declare_parameter('marker_near_forward_pulse_sec', 0.16)
        self.declare_parameter('marker_moving_forward_pulse_sec', 0.12)
        self.declare_parameter('map_pivot_pulse_sec', 0.20)
        self.declare_parameter('map_forward_pulse_sec', 0.25)
        self.declare_parameter('command_stop_pause_sec', 0.05)

    def load_follow_parameters(self):
        self.target_marker_id = int(
            self.get_parameter('target_marker_id').value
        )

        self.follow_distance = float(
            self.get_parameter('follow_distance').value
        )

        self.map_follow_center_distance = float(
            self.get_parameter('map_follow_center_distance').value
        )

        desired_camera_marker_distance = float(
            self.get_parameter('desired_camera_marker_distance').value
        )

        desired_bumper_gap = float(
            self.get_parameter('desired_bumper_gap').value
        )

        if desired_camera_marker_distance > 0.0:
            self.map_follow_center_distance = desired_camera_marker_distance

        elif desired_bumper_gap >= 0.0:
            self.map_follow_center_distance = (
                desired_bumper_gap
                + float(self.get_parameter('leader_pose_to_rear_marker_offset').value)
                + float(self.get_parameter('follower_pose_to_front_camera_offset').value)
            )

        else:
            self.map_follow_center_distance = self.follow_distance

        self.follow_distance = self.map_follow_center_distance

        self.leader_pose_to_rear_marker_offset = float(
            self.get_parameter('leader_pose_to_rear_marker_offset').value
        )

        self.follower_pose_to_front_camera_offset = float(
            self.get_parameter('follower_pose_to_front_camera_offset').value
        )

        self.marker_use_max_distance = float(
            self.get_parameter('marker_use_max_distance').value
        )

        self.marker_far_distance = float(
            self.get_parameter('marker_far_distance').value
        )

        self.marker_pulse_distance = float(
            self.get_parameter('marker_pulse_distance').value
        )

        self.marker_hold_min_distance = float(
            self.get_parameter('marker_hold_min_distance').value
        )

        self.marker_hold_max_distance = float(
            self.get_parameter('marker_hold_max_distance').value
        )

        self.marker_too_close_distance = float(
            self.get_parameter('marker_too_close_distance').value
        )

        self.marker_emergency_distance = float(
            self.get_parameter('marker_emergency_distance').value
        )

        self.marker_align_tolerance_deg = float(
            self.get_parameter('marker_align_tolerance_deg').value
        )

        self.marker_forward_limit_deg = float(
            self.get_parameter('marker_forward_limit_deg').value
        )

        self.marker_hard_stop_angle_deg = float(
            self.get_parameter('marker_hard_stop_angle_deg').value
        )

        self.map_arrive_distance = float(
            self.get_parameter('map_arrive_distance').value
        )

        self.map_yaw_tolerance_deg = float(
            self.get_parameter('map_yaw_tolerance_deg').value
        )

        self.map_hard_stop_angle_deg = float(
            self.get_parameter('map_hard_stop_angle_deg').value
        )

        self.marker_distance_filter_alpha = float(
            self.get_parameter('marker_distance_filter_alpha').value
        )

        self.target_marker_timeout_sec = float(
            self.get_parameter('target_marker_timeout_sec').value
        )

        self.follower_pose_timeout_sec = float(
            self.get_parameter('follower_pose_timeout_sec').value
        )

        self.leader_pose_timeout_sec = float(
            self.get_parameter('leader_pose_timeout_sec').value
        )

        self.marker_pose_distance_max_error = float(
            self.get_parameter('marker_pose_distance_max_error').value
        )

        self.marker_pose_mismatch_required_count = int(
            self.get_parameter('marker_pose_mismatch_required_count').value
        )

        self.mode_transition_stop_sec = float(
            self.get_parameter('mode_transition_stop_sec').value
        )

        self.follower_ahead_stop_distance = float(
            self.get_parameter('follower_ahead_stop_distance').value
        )

        self.follower_ahead_lateral_tolerance = float(
            self.get_parameter('follower_ahead_lateral_tolerance').value
        )

        self.follower_ahead_required_count = int(
            self.get_parameter('follower_ahead_required_count').value
        )

        self.leader_moving_speed = float(
            self.get_parameter('leader_moving_speed').value
        )

        self.leader_turning_speed = float(
            self.get_parameter('leader_turning_speed').value
        )

        self.marker_align_pulse_sec = float(
            self.get_parameter('marker_align_pulse_sec').value
        )

        self.marker_far_forward_pulse_sec = float(
            self.get_parameter('marker_far_forward_pulse_sec').value
        )

        self.marker_near_forward_pulse_sec = float(
            self.get_parameter('marker_near_forward_pulse_sec').value
        )

        self.marker_moving_forward_pulse_sec = float(
            self.get_parameter('marker_moving_forward_pulse_sec').value
        )

        self.map_pivot_pulse_sec = float(
            self.get_parameter('map_pivot_pulse_sec').value
        )

        self.map_forward_pulse_sec = float(
            self.get_parameter('map_forward_pulse_sec').value
        )

        self.command_stop_pause_sec = float(
            self.get_parameter('command_stop_pause_sec').value
        )

        self.filtered_marker_distance = None
        self.filtered_marker_bearing_deg = None

    def filter_marker_distance(self, raw_distance):
        if raw_distance <= 0.0:
            self.filtered_marker_distance = None
            return 0.0

        if self.filtered_marker_distance is None:
            self.filtered_marker_distance = raw_distance
        else:
            alpha = max(
                0.0,
                min(1.0, self.marker_distance_filter_alpha)
            )

            self.filtered_marker_distance = (
                alpha * raw_distance
                + (1.0 - alpha) * self.filtered_marker_distance
            )

        return self.filtered_marker_distance

    def filter_marker_bearing(self, raw_bearing_deg):
        if self.filtered_marker_bearing_deg is None:
            self.filtered_marker_bearing_deg = raw_bearing_deg
        else:
            alpha = max(
                0.0,
                min(1.0, self.marker_distance_filter_alpha)
            )

            self.filtered_marker_bearing_deg = (
                alpha * raw_bearing_deg
                + (1.0 - alpha) * self.filtered_marker_bearing_deg
            )

        return self.filtered_marker_bearing_deg

    def reset_marker_measurement(self):
        self.target_distance = 0.0
        self.target_bearing_deg = 0.0
        self.target_rel_x = 0.0
        self.target_rel_y = 0.0
        self.target_rel_z = 0.0
        self.target_offset_x = 0.0
        self.target_detected = False

        self.filtered_marker_distance = None
        self.filtered_marker_bearing_deg = None
        self.marker_pose_mismatch_count = 0

    def quaternion_to_yaw(
        self,
        x,
        y,
        z,
        w
    ):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def value_or_zero(self, value):
        if value is None:
            return 0.0

        return float(value)

    def destroy_node(self):
        self.publish_cmd('s')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = F1ToF2FollowNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.publish_cmd('s')

    finally:
        node.publish_cmd('s')
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
