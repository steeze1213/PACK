#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class SerialEncoderNode(Node):
    def __init__(self):
        super().__init__('serial_encoder_node')

        self.encoder_pub = self.create_publisher(String, '/encoder_counts', 10)

        self.cmd_sub = self.create_subscription(
            String,
            '/robot_cmd',
            self.cmd_callback,
            10
        )

        self.port = '/dev/ttyACM0'
        self.baudrate = 9600

        self.serial_conn = None
        self.connect_serial()

        self.timer = self.create_timer(0.02, self.read_serial)

    def connect_serial(self):
        while self.serial_conn is None:
            try:
                self.serial_conn = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=0.01
                )
                time.sleep(2.0)
                self.get_logger().info(f'Connected to Arduino: {self.port}')
            except Exception as e:
                self.get_logger().warn(f'Waiting for Arduino serial: {e}')
                time.sleep(1.0)

    def read_serial(self):
        if self.serial_conn is None:
            return

        try:
            line = self.serial_conn.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if not line:
                return

            if line.startswith('ENC,'):
                msg = String()
                msg.data = line
                self.encoder_pub.publish(msg)

            elif line.startswith('ACK,') or line.startswith('READY,') or line.startswith('CMD,'):
                self.get_logger().info(line)

        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.reconnect_serial()

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd not in ['w','a', 's', 'c', 'd', 'q', 'e']:
            self.get_logger().warn(f'Unknown command ignored: {cmd}')
            return

        if self.serial_conn is None:
            self.get_logger().warn('Serial is not connected.')
            return

        try:
            self.serial_conn.write(cmd.encode('utf-8'))
            self.get_logger().info(f'Sent command to Arduino: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.reconnect_serial()

    def reconnect_serial(self):
        try:
            if self.serial_conn is not None:
                self.serial_conn.close()
        except Exception:
            pass

        self.serial_conn = None
        self.connect_serial()


def main(args=None):
    rclpy.init(args=args)
    node = SerialEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

