import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odom_node')

        self.encoder_sub = self.create_subscription(
            String,
            '/encoder_counts',
            self.encoder_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            String,
            '/robot_cmd',
            self.cmd_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            String,
            '/robot_pose',
            10
        )

        self.wheel_circumference = 0.21
        self.track_width = 0.23
        self.counts_per_turn = 3600.0

        self.straight_deadband_counts = 80

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.get_logger().info('Encoder odometry node started.')
        self.get_logger().info(f'wheel_circumference: {self.wheel_circumference} m')
        self.get_logger().info(f'track_width: {self.track_width} m')
        self.get_logger().info(f'counts_per_turn: {self.counts_per_turn}')
        self.get_logger().info(f'straight_deadband_counts: {self.straight_deadband_counts}')

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd == 'c':
            self.reset_pose()
            self.get_logger().info('Pose reset by /robot_cmd c')

    def encoder_callback(self, msg):
        line = msg.data.strip()

        if not line.startswith('ENC,'):
            return

        try:
            parts = line.split(',')

            if len(parts) != 6:
                self.get_logger().warn(f'Invalid encoder data: {line}')
                return

            left_count = int(parts[1])
            right_count = int(parts[2])
            left_delta = int(parts[3])
            right_delta = int(parts[4])
            cmd = parts[5].strip()

            if cmd == 's':
                self.publish_pose(left_count, right_count, left_delta, right_delta, cmd)
                return

            left_abs_delta = abs(left_delta)
            right_abs_delta = abs(right_delta)

            left_distance = self.count_to_distance(left_abs_delta)
            right_distance = self.count_to_distance(right_abs_delta)

            forward_distance = (left_distance + right_distance) / 2.0

            delta_count_diff = right_abs_delta - left_abs_delta

            if abs(delta_count_diff) <= self.straight_deadband_counts:
                delta_yaw = 0.0
            else:
                delta_yaw = (right_distance - left_distance) / self.track_width

            mid_yaw = self.yaw + (delta_yaw / 2.0)

            self.x += forward_distance * math.cos(mid_yaw)
            self.y += forward_distance * math.sin(mid_yaw)
            self.yaw += delta_yaw

            self.yaw = self.normalize_angle(self.yaw)

            self.publish_pose(left_count, right_count, left_delta, right_delta, cmd)

        except Exception as e:
            self.get_logger().warn(f'Parse error: {line} / {e}')

    def count_to_distance(self, count):
        turns = count / self.counts_per_turn
        distance = turns * self.wheel_circumference
        return distance

    def reset_pose(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def publish_pose(self, left_count, right_count, left_delta, right_delta, cmd):
        yaw_deg = math.degrees(self.yaw)

        msg = String()
        msg.data = (
            f'POSE,'
            f'x={self.x:.4f},'
            f'y={self.y:.4f},'
            f'yaw={self.yaw:.4f},'
            f'yaw_deg={yaw_deg:.2f},'
            f'left_count={left_count},'
            f'right_count={right_count},'
            f'left_delta={left_delta},'
            f'right_delta={right_delta},'
            f'cmd={cmd}'
        )

        self.pose_pub.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = EncoderOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
