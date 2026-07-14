#!/usr/bin/env python3
"""Coordinate the final demo without taking over normal robot control."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionManagerNode(Node):
    """Translate dashboard commands and provide unified status topics."""

    def __init__(self):
        super().__init__('mission_manager_node')

        self.publishers_by_topic = {
            topic: self.create_publisher(String, topic, 10)
            for topic in (
                '/f1/mission_enable',
                '/f2/follow_enable',
                '/f1/robot_cmd',
                '/f2/robot_cmd',
                '/tb3/mission_enable',
                '/tb3/robot_cmd',
                '/arm_cmd',
                '/mission_status',
                '/f2/hybrid_status',
            )
        }
        self.create_subscription(
            String, '/mission_cmd', self.mission_command_callback, 10
        )
        self.create_subscription(
            String, '/f2/follow_status', self.f2_status_callback, 10
        )
        self.compatibility_subscriptions = []
        for topic in (
            '/f2/aruco_marker',
            '/f2/load_wait',
            '/f2/load_done',
            '/f2/return_home',
            '/f2/reanchor',
        ):
            callback = (
                self.ignore_f2_aruco_marker
                if topic == '/f2/aruco_marker'
                else self.f2_compatibility_callback
            )
            self.compatibility_subscriptions.append(
                self.create_subscription(String, topic, callback, 10)
            )

        self.state = 'IDLE'
        self.last_command = 'startup'
        self.last_update = time.monotonic()
        self.create_timer(0.5, self.publish_status)
        self.publish_stop(disable_controllers=True)
        self.get_logger().info(
            'Mission manager ready: /mission_cmd accepts start, pause, stop'
        )

    def publish(self, topic, data):
        msg = String()
        msg.data = data
        self.publishers_by_topic[topic].publish(msg)

    def mission_command_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ('start', 'resume'):
            self.publish('/f1/mission_enable', 'start')
            self.publish('/f2/follow_enable', 'start')
            self.publish('/tb3/mission_enable', 'start')
            self.state = 'RUNNING'
        elif command == 'pause':
            self.publish_stop(disable_controllers=True)
            self.state = 'PAUSED'
        elif command in ('stop', 'emergency_stop', 'estop'):
            self.publish_stop(disable_controllers=True)
            self.state = 'EMERGENCY_STOP'
        else:
            self.get_logger().warn(f'Unknown /mission_cmd ignored: {command}')
            return
        self.last_command = command
        self.last_update = time.monotonic()
        self.publish_status()

    def publish_stop(self, disable_controllers):
        if disable_controllers:
            self.publish('/f1/mission_enable', 'stop')
            self.publish('/f2/follow_enable', 'stop')
            self.publish('/tb3/mission_enable', 'stop')
        self.publish('/f1/robot_cmd', 's')
        self.publish('/f2/robot_cmd', 's')
        self.publish('/tb3/robot_cmd', 's')
        self.publish('/arm_cmd', 'stop')

    def f2_status_callback(self, msg):
        # Stable public topic while the final launch uses the legacy F2
        # controller. A future hybrid node can publish this topic directly.
        self.publish(
            '/f2/hybrid_status',
            f'HYBRID_STATUS,implementation=legacy_follow,detail={msg.data}',
        )

    def f2_compatibility_callback(self, msg):
        self.publish(
            '/f2/hybrid_status',
            'HYBRID_STATUS,implementation=legacy_follow,'
            f'compatibility_command={msg.data.strip()},supported=0',
        )

    @staticmethod
    def ignore_f2_aruco_marker(_msg):
        """Keep the future F2 hybrid input visible during legacy fallback."""

    def publish_status(self):
        age = time.monotonic() - self.last_update
        self.publish(
            '/mission_status',
            f'MISSION_STATUS,state={self.state},'
            f'last_cmd={self.last_command},age={age:.1f}',
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop(disable_controllers=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
