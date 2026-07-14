#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')

        self.declare_parameter('x', -0.037569766737571876)
        self.declare_parameter('y', 0.10523828207830883)
        self.declare_parameter('yaw', -0.007148110090032988)
        self.declare_parameter('delay_sec', 4.0)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('interval_sec', 0.5)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.delay_sec = float(self.get_parameter('delay_sec').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.interval_sec = float(self.get_parameter('interval_sec').value)

        self.sent_count = 0
        self.started = False

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.delay_timer = self.create_timer(self.delay_sec, self.start_publish)
        self.publish_timer = None

        self.get_logger().info(
            f'자동 초기 pose 대기 중: x={self.x}, y={self.y}, yaw={self.yaw}, '
            f'delay={self.delay_sec}s'
        )

    def start_publish(self):
        if self.started:
            return

        self.started = True
        self.delay_timer.cancel()

        self.get_logger().info('/initialpose 자동 발행 시작')

        self.publish_timer = self.create_timer(
            self.interval_sec,
            self.publish_initial_pose
        )

    def publish_initial_pose(self):
        if self.sent_count >= self.publish_count:
            if self.publish_timer is not None:
                self.publish_timer.cancel()

            self.get_logger().info('자동 초기 pose 발행 완료')
            return

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.pub.publish(msg)
        self.sent_count += 1

        self.get_logger().info(
            f'/initialpose 발행 {self.sent_count}/{self.publish_count}: '
            f'x={self.x}, y={self.y}, yaw={self.yaw}'
        )


def main():
    rclpy.init()
    node = AutoInitialPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
