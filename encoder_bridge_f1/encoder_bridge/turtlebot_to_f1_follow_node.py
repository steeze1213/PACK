#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class TurtlebotToF1FollowNode(Node):
    def __init__(self):
        super().__init__('turtlebot_to_f1_follow_node')

        self.relative_pose_sub = self.create_subscription(
            String,
            '/relative_pose',
            self.relative_pose_callback,
            10
        )

        self.target_marker_sub = self.create_subscription(
            String,
            '/target_marker',
            self.target_marker_callback,
            10
        )

        self.turtlebot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.turtlebot_pose_callback,
            10
        )

        self.turtlebot_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.turtlebot_odom_callback,
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

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.robot_pose_source = 'none'

        self.turtlebot_x = None
        self.turtlebot_y = None
        self.turtlebot_yaw = None

        self.turtlebot_linear_speed = 0.0
        self.turtlebot_angular_speed = 0.0

        self.target_detected = False

        self.target_rel_x = 0.0
        self.target_rel_y = 0.0
        self.target_rel_z = 0.0

        self.target_distance = 0.0
        self.target_bearing_deg = 0.0
        self.target_offset_x = 0.0

        self.last_robot_pose_time = None
        self.last_turtlebot_pose_time = None
        self.last_turtlebot_odom_time = None
        self.last_target_marker_time = None

        self.declare_follow_parameters()
        self.load_follow_parameters()

        self.turtlebot_moving_speed = 0.015

        self.marker_align_pulse_sec = 0.12
        self.marker_far_forward_pulse_sec = 0.18
        self.marker_near_forward_pulse_sec = 0.08
        self.marker_moving_forward_pulse_sec = 0.06

        self.map_pivot_pulse_sec = 0.18
        self.map_forward_pulse_sec = 0.15

        self.command_stop_pause_sec = 0.08

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

        self.get_logger().info(
            'turtlebot_to_f1_follow_node started.'
        )

        self.get_logger().info(
            f'Target marker ID: {self.target_marker_id}'
        )

        self.get_logger().info(
            f'Map follow center distance: '
            f'{self.map_follow_center_distance:.2f} m'
        )

        self.get_logger().info(
            f'Marker hold range: '
            f'{self.marker_hold_min_distance:.2f} m ~ '
            f'{self.marker_hold_max_distance:.2f} m'
        )

        self.get_logger().info(
            'Subscribe: /relative_pose, /target_marker, /amcl_pose, /odom'
        )

        self.get_logger().info(
            'Publish: /robot_cmd, /follow_status'
        )

        self.get_logger().info(
            'Follow is disabled. Publish start to /follow_enable.'
        )

    def relative_pose_callback(self, msg):
        parsed = self.parse_key_value_message(
            msg.data,
            'RELPOSE'
        )

        if parsed is None:
            return

        try:
            self.robot_x = float(
                parsed.get('x')
            )

            self.robot_y = float(
                parsed.get('y')
            )

            self.robot_yaw = float(
                parsed.get('yaw')
            )

            self.robot_pose_source = str(
                parsed.get(
                    'source',
                    'unknown'
                )
            )

        except (TypeError, ValueError):
            return

        self.last_robot_pose_time = (
            self.get_clock().now()
        )

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
            detected = int(
                parsed.get(
                    'detected',
                    0
                )
            )

        except (TypeError, ValueError):
            detected = 0

        if detected != 1:
            self.reset_marker_measurement()
            self.target_detected = False
            return

        try:
            marker_id = int(
                parsed.get(
                    'id',
                    -1
                )
            )

            if marker_id != self.target_marker_id:
                self.target_detected = False
                return

            self.target_rel_x = float(
                parsed.get(
                    'rel_x',
                    0.0
                )
            )

            self.target_rel_y = float(
                parsed.get(
                    'rel_y',
                    0.0
                )
            )

            self.target_rel_z = float(
                parsed.get(
                    'rel_z',
                    0.0
                )
            )

            raw_distance = float(
                parsed.get(
                    'planar_distance',
                    0.0
                )
            )
            self.target_distance = self.filter_marker_distance(
                raw_distance
            )

            raw_bearing_deg = float(
                parsed.get(
                    'bearing_yaw_deg',
                    0.0
                )
            )
            self.target_bearing_deg = self.filter_marker_bearing(
                raw_bearing_deg
            )

            self.target_offset_x = float(
                parsed.get(
                    'offset_x',
                    0.0
                )
            )

            self.target_detected = (
                self.target_distance > 0.0
            )

        except (TypeError, ValueError):
            self.reset_marker_measurement()
            self.target_detected = False

    def turtlebot_pose_callback(self, msg):
        self.turtlebot_x = float(
            msg.pose.pose.position.x
        )

        self.turtlebot_y = float(
            msg.pose.pose.position.y
        )

        orientation = (
            msg.pose.pose.orientation
        )

        self.turtlebot_yaw = (
            self.quaternion_to_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            )
        )

        self.last_turtlebot_pose_time = (
            self.get_clock().now()
        )

    def turtlebot_odom_callback(self, msg):
        self.turtlebot_linear_speed = float(
            msg.twist.twist.linear.x
        )

        self.turtlebot_angular_speed = float(
            msg.twist.twist.angular.z
        )

        self.last_turtlebot_odom_time = (
            self.get_clock().now()
        )

    def follow_enable_callback(self, msg):
        command = msg.data.strip().lower()

        if command in [
            'start',
            'on',
            'enable',
            '1'
        ]:
            self.follow_enabled = True
            self.follow_mode = 'WAITING'

            self.reset_motion_state()
            self.publish_cmd('s')

            self.get_logger().warn(
                'FOLLOW ENABLED'
            )

            return

        if command in [
            'stop',
            'off',
            'disable',
            '0'
        ]:
            self.follow_enabled = False
            self.follow_mode = 'DISABLED'

            self.reset_motion_state()
            self.publish_cmd('s')

            self.get_logger().warn(
                'FOLLOW DISABLED'
            )

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

        if not self.is_robot_pose_fresh(now):
            self.follow_mode = 'ROBOT_POSE_LOST'
            self.publish_cmd('s')
            self.log_warning_throttled(
                '4WD relative pose timeout. Stop.'
            )
            self.publish_status(now)
            return

        if (
            marker_available
            and self.target_distance > 0.0
            and self.target_distance
            <= self.marker_use_max_distance
        ):
            if not self.is_marker_pose_consistent():
                self.follow_mode = 'MARKER_POSE_MISMATCH'
                self.publish_cmd('s')
                self.log_warning_throttled(
                    'Marker distance and AMCL map distance mismatch. Stop.'
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

        if not self.is_turtlebot_pose_fresh(now):
            self.follow_mode = (
                'TURTLEBOT_POSE_LOST'
            )

            self.publish_cmd('s')

            self.log_warning_throttled(
                'Target marker lost and TurtleBot AMCL pose timeout. Stop.'
            )

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

        if (
            distance
            <= self.marker_emergency_distance
        ):
            self.follow_mode = (
                'EMERGENCY_TOO_CLOSE'
            )

            self.publish_cmd('s')

            self.log_status(
                f'EMERGENCY distance={distance:.3f}m'
            )

            return

        if (
            distance
            <= self.marker_too_close_distance
        ):
            self.follow_mode = 'TOO_CLOSE'

            self.publish_cmd('s')

            self.log_status(
                f'TOO_CLOSE distance={distance:.3f}m'
            )

            return

        if (
            abs(bearing_deg)
            >= self.marker_hard_stop_angle_deg
        ):
            self.follow_mode = (
                'MARKER_HARD_ALIGN'
            )

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
                f'MARKER_HARD_ALIGN '
                f'distance={distance:.3f} '
                f'bearing={bearing_deg:.1f} '
                f'cmd={command}'
            )

            return

        if (
            abs(bearing_deg)
            > self.marker_align_tolerance_deg
        ):
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
                f'MARKER_ALIGN '
                f'distance={distance:.3f} '
                f'bearing={bearing_deg:.1f} '
                f'cmd={command}'
            )

            return

        if (
            abs(bearing_deg)
            > self.marker_forward_limit_deg
        ):
            self.follow_mode = (
                'MARKER_FORWARD_BLOCKED'
            )

            self.publish_cmd('s')

            self.log_status(
                f'FORWARD_BLOCKED '
                f'distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )

            return

        if distance >= self.marker_far_distance:
            self.follow_mode = (
                'MARKER_FORWARD_FAR'
            )

            self.start_command(
                'w',
                self.marker_far_forward_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_FORWARD_FAR '
                f'distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )

            return

        if (
            distance
            >= self.marker_pulse_distance
        ):
            self.follow_mode = (
                'MARKER_FORWARD_PULSE'
            )

            self.start_command(
                'w',
                self.marker_near_forward_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_FORWARD_PULSE '
                f'distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )

            return

        turtlebot_moving = (
            self.is_turtlebot_moving(now)
        )

        if (
            distance > 0.11
            and turtlebot_moving
        ):
            self.follow_mode = (
                'MARKER_MOVING_PULSE'
            )

            self.start_command(
                'w',
                self.marker_moving_forward_pulse_sec,
                now
            )

            self.log_status(
                f'MARKER_MOVING_PULSE '
                f'distance={distance:.3f} '
                f'tb_speed={self.turtlebot_linear_speed:.3f}'
            )

            return

        if (
            self.marker_hold_min_distance
            <= distance
            <= self.marker_hold_max_distance
        ):
            self.follow_mode = 'MARKER_HOLD'

            self.publish_cmd('s')

            self.log_status(
                f'MARKER_HOLD '
                f'distance={distance:.3f} '
                f'bearing={bearing_deg:.1f}'
            )

            return

        self.follow_mode = 'MARKER_STOP'

        self.publish_cmd('s')

        self.log_status(
            f'MARKER_STOP distance={distance:.3f}'
        )

    def run_map_follow(self, now):
        leader_to_follower_along_heading = (
            (self.robot_x - self.turtlebot_x)
            * math.cos(self.turtlebot_yaw)
            + (self.robot_y - self.turtlebot_y)
            * math.sin(self.turtlebot_yaw)
        )

        lateral_error = (
            -(self.robot_x - self.turtlebot_x)
            * math.sin(self.turtlebot_yaw)
            + (self.robot_y - self.turtlebot_y)
            * math.cos(self.turtlebot_yaw)
        )

        if self.is_follower_ahead_of_leader(
            leader_to_follower_along_heading,
            lateral_error
        ):
            self.follow_mode = 'FOLLOWER_NOT_BEHIND_LEADER'
            self.publish_cmd('s')
            self.log_status(
                f'FOLLOWER_NOT_BEHIND_LEADER '
                f'along={leader_to_follower_along_heading:.3f} '
                f'lateral={lateral_error:.3f}'
            )
            return

        follow_x = (
            self.turtlebot_x
            - self.map_follow_center_distance
            * math.cos(self.turtlebot_yaw)
        )

        follow_y = (
            self.turtlebot_y
            - self.map_follow_center_distance
            * math.sin(self.turtlebot_yaw)
        )

        dx = follow_x - self.robot_x
        dy = follow_y - self.robot_y

        distance = math.hypot(
            dx,
            dy
        )

        target_yaw = math.atan2(
            dy,
            dx
        )

        yaw_error = self.normalize_angle(
            target_yaw - self.robot_yaw
        )

        yaw_error_deg = math.degrees(
            yaw_error
        )

        if distance <= self.map_arrive_distance:
            self.follow_mode = 'MAP_HOLD'

            self.publish_cmd('s')

            self.log_status(
                f'MAP_HOLD '
                f'distance={distance:.3f} '
                f'follow=({follow_x:.3f},{follow_y:.3f})'
            )

            return

        if (
            abs(yaw_error_deg)
            >= self.map_hard_stop_angle_deg
        ):
            self.follow_mode = (
                'MAP_REVERSE_DIRECTION'
            )

            self.publish_cmd('s')

            self.log_status(
                f'MAP_REVERSE_DIRECTION '
                f'distance={distance:.3f} '
                f'yaw_error={yaw_error_deg:.1f}'
            )

            return

        if (
            abs(yaw_error_deg)
            > self.map_yaw_tolerance_deg
        ):
            self.follow_mode = 'MAP_ALIGN'

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
                f'MAP_ALIGN '
                f'distance={distance:.3f} '
                f'yaw_error={yaw_error_deg:.1f} '
                f'cmd={command}'
            )

            return

        self.follow_mode = 'MAP_FORWARD'

        self.start_command(
            'w',
            self.map_forward_pulse_sec,
            now
        )

        self.log_status(
            f'MAP_FORWARD '
            f'distance={distance:.3f} '
            f'yaw_error={yaw_error_deg:.1f} '
            f'follow=({follow_x:.3f},{follow_y:.3f})'
        )

    def start_command(
        self,
        command,
        duration_sec,
        now
    ):
        if command not in [
            'w',
            'q',
            'e'
        ]:
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

        elapsed = (
            now - self.motion_start_time
        ).nanoseconds / 1e9

        if elapsed >= self.motion_duration_sec:
            self.publish_cmd('s')

            self.motion_state = 'STOP_PAUSE'
            self.stop_start_time = now

            return

        self.publish_cmd(
            self.active_cmd
        )

    def handle_stop_pause(self, now):
        self.publish_cmd('s')

        if self.stop_start_time is None:
            self.reset_motion_state()
            return

        elapsed = (
            now - self.stop_start_time
        ).nanoseconds / 1e9

        if (
            elapsed
            >= self.command_stop_pause_sec
        ):
            self.reset_motion_state()

    def reset_motion_state(self):
        self.motion_state = 'IDLE'
        self.active_cmd = 's'

        self.motion_start_time = None
        self.motion_duration_sec = 0.0
        self.stop_start_time = None

    def is_robot_pose_fresh(self, now):
        if (
            self.robot_x is None
            or self.robot_y is None
            or self.robot_yaw is None
            or self.last_robot_pose_time is None
        ):
            return False

        elapsed = (
            now - self.last_robot_pose_time
        ).nanoseconds / 1e9

        return (
            elapsed
            <= self.robot_pose_timeout_sec
        )

    def is_turtlebot_pose_fresh(self, now):
        if (
            self.turtlebot_x is None
            or self.turtlebot_y is None
            or self.turtlebot_yaw is None
            or self.last_turtlebot_pose_time is None
        ):
            return False

        elapsed = (
            now - self.last_turtlebot_pose_time
        ).nanoseconds / 1e9

        return (
            elapsed
            <= self.turtlebot_pose_timeout_sec
        )

    def is_target_marker_fresh(self, now):
        if (
            not self.target_detected
            or self.last_target_marker_time is None
        ):
            return False

        elapsed = (
            now - self.last_target_marker_time
        ).nanoseconds / 1e9

        return (
            elapsed
            <= self.target_marker_timeout_sec
        )

    def is_marker_pose_consistent(self):
        if (
            not self.is_robot_pose_fresh(self.get_clock().now())
            or not self.is_turtlebot_pose_fresh(self.get_clock().now())
        ):
            return True

        map_distance = math.hypot(
            self.turtlebot_x - self.robot_x,
            self.turtlebot_y - self.robot_y
        )

        expected_map_distance = (
            self.target_distance
            + self.leader_pose_to_rear_marker_offset
            + self.follower_pose_to_front_camera_offset
        )

        if (
            abs(map_distance - expected_map_distance)
            <= self.marker_pose_distance_max_error
        ):
            self.marker_pose_mismatch_count = 0
            return True

        self.marker_pose_mismatch_count += 1

        return (
            self.marker_pose_mismatch_count
            < self.marker_pose_mismatch_required_count
        )

    def is_follower_ahead_of_leader(self, along_heading, lateral_error):
        if (
            along_heading <= self.follower_ahead_stop_distance
            or abs(lateral_error) > self.follower_ahead_lateral_tolerance
        ):
            self.follower_ahead_count = 0
            return False

        self.follower_ahead_count += 1

        return (
            self.follower_ahead_count
            >= self.follower_ahead_required_count
        )

    def should_abort_active_command(self, now):
        if self.last_control_source == 'MARKER':
            if not self.is_robot_pose_fresh(now):
                self.follow_mode = 'ROBOT_POSE_LOST'
                return True

            if not self.is_target_marker_fresh(now):
                self.follow_mode = 'MARKER_LOST_DURING_COMMAND'
                return True

            if self.target_distance <= self.marker_too_close_distance:
                self.follow_mode = 'TOO_CLOSE'
                return True

        if self.last_control_source == 'MAP':
            if not self.is_robot_pose_fresh(now):
                self.follow_mode = 'ROBOT_POSE_LOST'
                return True

            if not self.is_turtlebot_pose_fresh(now):
                self.follow_mode = 'TURTLEBOT_POSE_LOST'
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

    def is_turtlebot_moving(self, now):
        if self.last_turtlebot_odom_time is None:
            return False

        elapsed = (
            now - self.last_turtlebot_odom_time
        ).nanoseconds / 1e9

        if (
            elapsed
            > self.turtlebot_odom_timeout_sec
        ):
            return False

        return (
            abs(self.turtlebot_linear_speed)
            >= self.turtlebot_moving_speed
            or abs(self.turtlebot_angular_speed)
            >= 0.05
        )

    def publish_cmd(self, command):
        if command not in [
            'w',
            's',
            'q',
            'e'
        ]:
            command = 's'

        msg = String()
        msg.data = command

        self.cmd_pub.publish(msg)

        self.last_cmd = command

    def publish_status(self, now):
        elapsed = (
            now - self.last_status_time
        ).nanoseconds / 1e9

        if elapsed < 0.25:
            return

        self.last_status_time = now

        marker_fresh = (
            self.is_target_marker_fresh(now)
        )

        robot_pose_fresh = (
            self.is_robot_pose_fresh(now)
        )

        turtlebot_pose_fresh = (
            self.is_turtlebot_pose_fresh(now)
        )

        status = (
            f'FOLLOW_STATUS,'
            f'enabled={1 if self.follow_enabled else 0},'
            f'mode={self.follow_mode},'
            f'cmd={self.last_cmd},'
            f'motion_state={self.motion_state},'
            f'robot_pose_fresh={1 if robot_pose_fresh else 0},'
            f'turtlebot_pose_fresh={1 if turtlebot_pose_fresh else 0},'
            f'target_detected={1 if marker_fresh else 0},'
            f'target_distance={self.target_distance:.3f},'
            f'target_bearing_deg={self.target_bearing_deg:.2f},'
            f'target_rel_x={self.target_rel_x:.3f},'
            f'target_rel_z={self.target_rel_z:.3f},'
            f'robot_x={self.value_or_zero(self.robot_x):.3f},'
            f'robot_y={self.value_or_zero(self.robot_y):.3f},'
            f'robot_yaw={self.value_or_zero(self.robot_yaw):.4f},'
            f'turtlebot_x={self.value_or_zero(self.turtlebot_x):.3f},'
            f'turtlebot_y={self.value_or_zero(self.turtlebot_y):.3f},'
            f'turtlebot_yaw={self.value_or_zero(self.turtlebot_yaw):.4f},'
            f'turtlebot_linear={self.turtlebot_linear_speed:.3f},'
            f'turtlebot_angular={self.turtlebot_angular_speed:.3f}'
        )

        msg = String()
        msg.data = status

        self.status_pub.publish(msg)

    def log_status(self, message):
        now = self.get_clock().now()

        elapsed = (
            now - self.last_log_time
        ).nanoseconds / 1e9

        if elapsed < 0.30:
            return

        self.last_log_time = now

        self.get_logger().info(
            f'{message}, '
            f'mode={self.follow_mode}, '
            f'cmd={self.last_cmd}, '
            f'robot_pose_source={self.robot_pose_source}'
        )

    def log_warning_throttled(self, message):
        now = self.get_clock().now()

        elapsed = (
            now - self.last_warning_time
        ).nanoseconds / 1e9

        if elapsed < 1.0:
            return

        self.last_warning_time = now

        self.get_logger().warn(
            message
        )

    def parse_key_value_message(
        self,
        data,
        prefix
    ):
        if not data.startswith(
            prefix + ','
        ):
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

            result[key.strip()] = (
                value.strip()
            )

        return result

    def declare_follow_parameters(self):
        self.declare_parameter('target_marker_id', 98)
        self.declare_parameter('follow_distance', 0.50)
        self.declare_parameter('map_follow_center_distance', 0.50)
        self.declare_parameter('leader_pose_to_rear_marker_offset', 0.0)
        self.declare_parameter('follower_pose_to_front_camera_offset', 0.0)
        self.declare_parameter('desired_bumper_gap', -1.0)
        self.declare_parameter('desired_camera_marker_distance', 0.0)
        self.declare_parameter('marker_far_distance', 0.45)
        self.declare_parameter('marker_pulse_distance', 0.35)
        self.declare_parameter('marker_hold_min_distance', 0.27)
        self.declare_parameter('marker_hold_max_distance', 0.33)
        self.declare_parameter('marker_too_close_distance', 0.22)
        self.declare_parameter('marker_emergency_distance', 0.17)
        self.declare_parameter('map_arrive_distance', 0.15)
        self.declare_parameter('marker_distance_filter_alpha', 0.35)
        self.declare_parameter('target_marker_timeout_sec', 0.60)
        self.declare_parameter('marker_pose_distance_max_error', 0.50)
        self.declare_parameter('marker_pose_mismatch_required_count', 3)
        self.declare_parameter('mode_transition_stop_sec', 0.15)
        self.declare_parameter('follower_ahead_stop_distance', 0.10)
        self.declare_parameter('follower_ahead_lateral_tolerance', 0.35)
        self.declare_parameter('follower_ahead_required_count', 3)

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
            self.map_follow_center_distance = (
                desired_camera_marker_distance
            )
        elif desired_bumper_gap >= 0.0:
            self.map_follow_center_distance = (
                desired_bumper_gap
                + float(
                    self.get_parameter(
                        'leader_pose_to_rear_marker_offset'
                    ).value
                )
                + float(
                    self.get_parameter(
                        'follower_pose_to_front_camera_offset'
                    ).value
                )
            )
        else:
            self.map_follow_center_distance = self.follow_distance

        self.follow_distance = self.map_follow_center_distance
        self.leader_pose_to_rear_marker_offset = float(
            self.get_parameter(
                'leader_pose_to_rear_marker_offset'
            ).value
        )
        self.follower_pose_to_front_camera_offset = float(
            self.get_parameter(
                'follower_pose_to_front_camera_offset'
            ).value
        )

        self.robot_pose_timeout_sec = 0.80
        self.turtlebot_pose_timeout_sec = 1.00
        self.turtlebot_odom_timeout_sec = 1.00
        self.target_marker_timeout_sec = float(
            self.get_parameter('target_marker_timeout_sec').value
        )

        self.marker_use_max_distance = 1.50
        self.marker_align_tolerance_deg = 10.0
        self.marker_forward_limit_deg = 18.0
        self.marker_hard_stop_angle_deg = 25.0
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
        self.map_arrive_distance = float(
            self.get_parameter('map_arrive_distance').value
        )
        self.map_yaw_tolerance_deg = 15.0
        self.map_hard_stop_angle_deg = 150.0
        self.marker_distance_filter_alpha = float(
            self.get_parameter('marker_distance_filter_alpha').value
        )
        self.marker_pose_distance_max_error = float(
            self.get_parameter('marker_pose_distance_max_error').value
        )
        self.marker_pose_mismatch_required_count = int(
            self.get_parameter(
                'marker_pose_mismatch_required_count'
            ).value
        )
        self.mode_transition_stop_sec = float(
            self.get_parameter('mode_transition_stop_sec').value
        )
        self.follower_ahead_stop_distance = float(
            self.get_parameter('follower_ahead_stop_distance').value
        )
        self.follower_ahead_lateral_tolerance = float(
            self.get_parameter(
                'follower_ahead_lateral_tolerance'
            ).value
        )
        self.follower_ahead_required_count = int(
            self.get_parameter('follower_ahead_required_count').value
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
        siny_cosp = 2.0 * (
            w * z
            + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y
            + z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

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

    node = TurtlebotToF1FollowNode()

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
