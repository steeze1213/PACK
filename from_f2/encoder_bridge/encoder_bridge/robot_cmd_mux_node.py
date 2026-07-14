#!/usr/bin/env python3
"""Mode-gated command mux for waypoint and follow/final-align controllers."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotCmdMuxNode(Node):
    VALID_COMMANDS = ('w', 's', 'q', 'e')
    START_COMMANDS = ('start', 'on', 'enable', '1')
    STOP_COMMANDS = ('stop', 'off', 'disable', '0')

    def __init__(self):
        super().__init__('robot_cmd_mux_node')

        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('localization_timeout_sec', 0.80)
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.localization_timeout_sec = float(
            self.get_parameter('localization_timeout_sec').value
        )
        goal_pose_topic = str(self.get_parameter('goal_pose_topic').value)

        self.mode = 'HOLD'
        self.waypoint_cmd = 's'
        self.follow_cmd = 's'
        self.waypoint_cmd_time = 0.0
        self.follow_cmd_time = 0.0
        self.last_output = 's'
        self.localization_received = False
        self.localization_fault = True
        self.localization_drive_allowed = False
        self.localization_reason = 'status_missing'
        self.localization_status_time = 0.0
        self.selected_source = 'none'

        self.create_subscription(
            String, '/waypoint_cmd', self.waypoint_cmd_callback, 20
        )
        self.create_subscription(
            String, '/follow_cmd', self.follow_cmd_callback, 20
        )
        self.create_subscription(
            String, '/waypoint_enable', self.waypoint_enable_callback, 10
        )
        self.create_subscription(
            String, '/follow_enable', self.follow_enable_callback, 10
        )
        self.create_subscription(
            String, '/final_align_enable', self.final_align_enable_callback, 10
        )
        self.create_subscription(
            String,
            '/localization_status',
            self.localization_status_callback,
            20
        )

        self.cmd_pub = self.create_publisher(String, '/robot_cmd', 20)
        self.status_pub = self.create_publisher(String, '/cmd_mux_status', 10)
        self.goal_clear_pub = self.create_publisher(String, goal_pose_topic, 10)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.25, self.publish_status)

        self.publish_output('s')
        self.get_logger().info(
            'robot_cmd_mux_node started in HOLD; explicit waypoint/follow enable required'
        )

    def waypoint_cmd_callback(self, msg):
        self.waypoint_cmd = self.sanitize(msg.data)
        self.waypoint_cmd_time = time.monotonic()

    def follow_cmd_callback(self, msg):
        self.follow_cmd = self.sanitize(msg.data)
        self.follow_cmd_time = time.monotonic()

    def waypoint_enable_callback(self, msg):
        command = msg.data.strip().lower()
        if command in self.START_COMMANDS:
            self.switch_mode('WAYPOINT')
        elif command in self.STOP_COMMANDS and self.mode == 'WAYPOINT':
            self.switch_mode('HOLD')

    def follow_enable_callback(self, msg):
        command = msg.data.strip().lower()
        if command in self.START_COMMANDS:
            self.switch_mode('FOLLOW', clear_goal=True)
        elif command in self.STOP_COMMANDS and self.mode == 'FOLLOW':
            self.switch_mode('HOLD')

    def final_align_enable_callback(self, msg):
        command = msg.data.strip().lower()
        if command in self.START_COMMANDS:
            self.switch_mode('FOLLOW', clear_goal=True)
        elif command in self.STOP_COMMANDS and self.mode == 'FOLLOW':
            self.switch_mode('HOLD')

    def localization_status_callback(self, msg):
        parsed = self.parse_key_value_message(
            msg.data, 'LOCALIZATION_STATUS'
        )
        if parsed is None:
            return
        try:
            self.localization_fault = int(parsed.get('fault', 1)) == 1
            self.localization_drive_allowed = (
                int(parsed.get('drive_allowed', 0)) == 1
            )
        except (TypeError, ValueError):
            self.localization_fault = True
            self.localization_drive_allowed = False
        self.localization_reason = str(parsed.get('reason', 'unknown'))
        self.localization_received = True
        self.localization_status_time = time.monotonic()

        # NOTE: we intentionally do NOT force-switch out of WAYPOINT on a
        # localization fault here. The control_loop already publishes 's' whenever
        # the fault is active (robot stopped), and staying in WAYPOINT lets motion
        # resume the instant drive_allowed returns -- otherwise a single momentary
        # drive_allowed=0 blip would drop WAYPOINT to HOLD and require a fresh
        # waypoint_enable, making driving on marginal localization impossible.
        # Safety is unchanged: the robot still moves ONLY while drive_allowed=1.

    def switch_mode(self, mode, clear_goal=False):
        if mode not in ('HOLD', 'WAYPOINT', 'FOLLOW'):
            mode = 'HOLD'
        self.mode = mode
        self.publish_output('s')

        if clear_goal:
            msg = String()
            msg.data = 'GOAL_CLEAR'
            self.goal_clear_pub.publish(msg)

        self.get_logger().warn(f'Command source switched to {self.mode}')

    def control_loop(self):
        now = time.monotonic()
        command = 's'
        self.selected_source = 'none'

        if self.mode == 'WAYPOINT':
            # WAYPOINT is map-based motion: it REQUIRES healthy wall localization to
            # MOVE. On a fault we publish 's' (robot stopped) but STAY in WAYPOINT so
            # motion resumes automatically when drive_allowed returns; we do not drop
            # to HOLD on transient blips. The robot still moves only when the fault
            # is clear (drive_allowed=1), so the safety requirement is preserved.
            if self.safety_fault_active(now):
                self.selected_source = 'wp_fault_stop'
                self.publish_output('s')
                return
            if now - self.waypoint_cmd_time <= self.command_timeout_sec:
                command = self.waypoint_cmd
                self.selected_source = 'waypoint'
        elif self.mode == 'FOLLOW':
            # FOLLOW is target-relative and does NOT require wall localization here.
            # The follow controller already fail-closes follow_cmd to 's' when the
            # target marker is lost / stale / at an abnormal range, and the command
            # timeout below stops us if follow_cmd goes stale (controller silent /
            # crashed). sanitize() forces any invalid command to 's'. So FOLLOW is
            # gated on TARGET validity (upstream) + freshness, not wall localization.
            if now - self.follow_cmd_time <= self.command_timeout_sec:
                command = self.follow_cmd
                self.selected_source = 'follow'
            else:
                self.selected_source = 'follow_timeout'

        self.publish_output(command)

    def publish_output(self, command):
        msg = String()
        msg.data = self.sanitize(command)
        self.cmd_pub.publish(msg)
        self.last_output = msg.data

    def publish_status(self):
        fault = self.safety_fault_active()
        msg = String()
        msg.data = (
            f'MUX,mode={self.mode},cmd={self.last_output},'
            f'fault={1 if fault else 0},source={self.selected_source},'
            f'localization_reason={self.localization_reason},'
            f'waypoint_cmd={self.waypoint_cmd},follow_cmd={self.follow_cmd}'
        )
        self.status_pub.publish(msg)

    def safety_fault_active(self, now=None):
        if now is None:
            now = time.monotonic()
        status_stale = (
            not self.localization_received
            or now - self.localization_status_time
            > self.localization_timeout_sec
        )
        return (
            status_stale
            or self.localization_fault
            or not self.localization_drive_allowed
        )

    @staticmethod
    def parse_key_value_message(data, prefix):
        if not data.startswith(prefix + ','):
            return None
        result = {}
        for part in data.split(',')[1:]:
            if '=' not in part:
                continue
            key, value = part.split('=', 1)
            result[key.strip()] = value.strip()
        return result

    def sanitize(self, command):
        command = str(command).strip().lower()
        return command if command in self.VALID_COMMANDS else 's'


def main(args=None):
    rclpy.init(args=args)
    node = RobotCmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_output('s')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
