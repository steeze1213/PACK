#!/usr/bin/env python3
"""Exclusive command router for the F1 unified controller graph.

Modes:
  paused      : hold stop
  follow      : route /f1/follow_cmd -> /f1/robot_cmd
  waypoint    : route /f1/waypoint_cmd -> /f1/robot_cmd
  return_home : route /f1/follow_cmd -> /f1/robot_cmd and trigger return_home
  load_exit   : route /f1/load_exit_cmd -> /f1/robot_cmd
  stop        : hold stop

load_exit is used only after loading is complete.
It is not the same as the hybrid follow node's POSE_RECOVERY state.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class F1ModeManagerNode(Node):
    VALID_MODES = (
        'paused',
        'follow',
        'waypoint',
        'return_home',
        'load_exit',
        'stop',
    )

    VALID_COMMANDS = (
        'w',
        'a',
        's',
        'c',
        'd',
        'q',
        'e',
    )

    def __init__(self):
        super().__init__('f1_mode_manager_node')

        self.mode = 'paused'
        self.last_output = 's'
        self.last_input = 'startup'
        self.last_input_time = time.monotonic()
        self.return_home_pending_at = None

        self.robot_cmd_pub = self.create_publisher(
            String,
            '/f1/robot_cmd',
            20,
        )

        self.status_pub = self.create_publisher(
            String,
            '/f1/mode_status',
            10,
        )

        self.hybrid_enable_pub = self.create_publisher(
            String,
            '/f1/mission_enable',
            10,
        )

        self.return_home_pub = self.create_publisher(
            String,
            '/f1/return_home',
            10,
        )

        self.create_subscription(
            String,
            '/f1/mode',
            self.mode_callback,
            10,
        )

        self.create_subscription(
            String,
            '/f1/follow_cmd',
            self.follow_command_callback,
            20,
        )

        self.create_subscription(
            String,
            '/f1/waypoint_cmd',
            self.waypoint_command_callback,
            20,
        )

        self.create_subscription(
            String,
            '/f1/load_exit_cmd',
            self.load_exit_command_callback,
            20,
        )

        self.create_timer(0.1, self.safety_timer)
        self.create_timer(0.5, self.publish_status)

        self.set_hybrid_enabled(False)
        self.publish_robot_command('s', 'startup paused')

        self.get_logger().info(
            'F1 mode manager started in paused mode. '
            'Available modes: paused/follow/waypoint/return_home/load_exit/stop'
        )

    @staticmethod
    def publish_text(publisher, data):
        msg = String()
        msg.data = data
        publisher.publish(msg)

    def mode_callback(self, msg):
        requested = msg.data.strip().lower()

        if requested not in self.VALID_MODES:
            self.get_logger().warn(f'Invalid F1 mode ignored: {requested}')
            return

        previous = self.mode
        self.mode = requested

        self.publish_robot_command(
            's',
            f'mode change {previous}->{requested}',
        )

        if requested in ('follow', 'return_home'):
            self.set_hybrid_enabled(True)
        else:
            self.set_hybrid_enabled(False)

        if requested == 'return_home':
            self.return_home_pending_at = time.monotonic() + 0.2
        else:
            self.return_home_pending_at = None

        self.get_logger().warn(f'F1 mode changed: {previous} -> {requested}')
        self.publish_status()

    def follow_command_callback(self, msg):
        if self.mode in ('follow', 'return_home'):
            self.route_command(msg.data, 'follow_cmd')

    def waypoint_command_callback(self, msg):
        if self.mode == 'waypoint':
            self.route_command(msg.data, 'waypoint_cmd')

    def load_exit_command_callback(self, msg):
        if self.mode == 'load_exit':
            self.route_command(msg.data, 'load_exit_cmd')

    def route_command(self, raw_command, source):
        command = raw_command.strip().lower()

        if command not in self.VALID_COMMANDS:
            self.get_logger().warn(
                f'Invalid command from {source} ignored: {command}'
            )
            return

        self.publish_robot_command(command, source)

    def publish_robot_command(self, command, source):
        self.publish_text(self.robot_cmd_pub, command)

        self.last_output = command
        self.last_input = source
        self.last_input_time = time.monotonic()

    def set_hybrid_enabled(self, enabled):
        self.publish_text(
            self.hybrid_enable_pub,
            'start' if enabled else 'stop',
        )

    def safety_timer(self):
        if (
            self.return_home_pending_at is not None
            and time.monotonic() >= self.return_home_pending_at
        ):
            self.publish_text(self.return_home_pub, 'start')
            self.return_home_pending_at = None

        if self.mode in ('paused', 'stop'):
            self.publish_robot_command('s', f'{self.mode} hold')

    def publish_status(self):
        age = time.monotonic() - self.last_input_time

        status = (
            f'MODE_STATUS,mode={self.mode},cmd={self.last_output},'
            f'source={self.last_input},age={age:.2f}'
        )

        self.publish_text(self.status_pub, status)

    def shutdown(self):
        for _ in range(3):
            self.publish_robot_command('s', 'shutdown')


def main(args=None):
    rclpy.init(args=args)

    node = F1ModeManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
