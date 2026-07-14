import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ArucoFollowNode(Node):
    def __init__(self):
        super().__init__('aruco_follow_node')

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

        self.center_deadband = 40

        self.last_cmd = None

        self.get_logger().info('Aruco follow node started.')
        self.get_logger().info(f'center_deadband: {self.center_deadband}')

    def aruco_callback(self, msg):
        data = msg.data.strip()

        parsed = self.parse_aruco_data(data)

        if parsed is None:
            self.get_logger().warn(f'Invalid aruco data: {data}')
            return

        detected = parsed.get('detected', 0)
        marker_id = parsed.get('id', -1)
        offset_x = parsed.get('offset_x', 0)

        if detected == 0 or marker_id == -1:
            cmd = 's'
            reason = 'marker not detected'
        else:
            if offset_x < -self.center_deadband:
                cmd = 'a'
                reason = f'marker left offset_x={offset_x}'
            elif offset_x > self.center_deadband:
                cmd = 'd'
                reason = f'marker right offset_x={offset_x}'
            else:
                cmd = 'w'
                reason = f'marker centered offset_x={offset_x}'

        self.publish_cmd(cmd, reason)

    def parse_aruco_data(self, data):
        if not data.startswith('ARUCO,'):
            return None

        result = {}

        parts = data.split(',')

        for part in parts[1:]:
            if '=' not in part:
                continue

            key, value = part.split('=', 1)
            key = key.strip()
            value = value.strip()

            try:
                result[key] = int(value)
            except ValueError:
                try:
                    result[key] = float(value)
                except ValueError:
                    result[key] = value

        return result

    def publish_cmd(self, cmd, reason):
        if cmd == self.last_cmd:
            return

        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

        self.last_cmd = cmd

        self.get_logger().info(f'CMD={cmd} | {reason}')


def main(args=None):
    rclpy.init(args=args)

    node = ArucoFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
