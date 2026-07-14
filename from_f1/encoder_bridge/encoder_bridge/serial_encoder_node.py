#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialEncoderNode(Node):
    def __init__(self):
        super().__init__('serial_encoder_node')

        self.encoder_pub = self.create_publisher(
            String,
            '/encoder_counts',
            10
        )

        self.cmd_sub = self.create_subscription(
            String,
            '/robot_cmd',
            self.cmd_callback,
            10
        )

        self.port = '/dev/ttyACM0'
        self.baudrate = 9600

        self.serial_conn = None

        self.last_sent_cmd = None
        self.last_sent_time = 0.0
        self.last_logged_cmd = None

        self.stop_repeat_interval_sec = 1.50
        self.move_repeat_interval_sec = 0.15

        self.last_ack_line = None
        self.last_ack_log_time = 0.0
        self.ack_log_interval_sec = 3.0

        self.connect_serial()

        self.timer = self.create_timer(
            0.02,
            self.read_serial
        )

        self.get_logger().info(
            'serial_encoder_node started. Duplicate commands and ACK logs are throttled.'
        )

    def connect_serial(self):
        while self.serial_conn is None:
            try:
                self.serial_conn = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=0.01
                )
                time.sleep(2.0)
                self.get_logger().info(
                    f'Connected to Arduino: {self.port}'
                )
            except Exception as error:
                self.get_logger().warn(
                    f'Waiting for Arduino serial: {error}'
                )
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
                return

            if (
                line.startswith('ACK,')
                or line.startswith('READY,')
                or line.startswith('CMD,')
            ):
                self.log_ack_throttled(line)
                return

        except Exception as error:
            self.get_logger().error(
                f'Serial read error: {error}'
            )
            self.reconnect_serial()

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd not in ['w', 'a', 's', 'c', 'd', 'q', 'e']:
            self.get_logger().warn(
                f'Unknown command ignored: {cmd}'
            )
            return

        if self.serial_conn is None:
            self.get_logger().warn(
                'Serial is not connected.'
            )
            return

        if not self.should_send_command(cmd):
            return

        try:
            self.serial_conn.write(cmd.encode('utf-8'))
            self.last_sent_cmd = cmd
            self.last_sent_time = time.monotonic()
            self.log_command_throttled(cmd)
        except Exception as error:
            self.get_logger().error(
                f'Serial write error: {error}'
            )
            self.reconnect_serial()

    def should_send_command(self, cmd):
        now = time.monotonic()

        if cmd != self.last_sent_cmd:
            return True

        elapsed = now - self.last_sent_time

        if cmd == 's':
            return elapsed >= self.stop_repeat_interval_sec

        return elapsed >= self.move_repeat_interval_sec

    def log_command_throttled(self, cmd):
        if cmd != self.last_logged_cmd:
            self.get_logger().info(
                f'Sent command to Arduino: {cmd}'
            )
            self.last_logged_cmd = cmd
            return

        if cmd != 's':
            self.get_logger().debug(
                f'Resent command to Arduino: {cmd}'
            )

    def log_ack_throttled(self, line):
        now = time.monotonic()

        if (
            line != self.last_ack_line
            or now - self.last_ack_log_time >= self.ack_log_interval_sec
        ):
            self.get_logger().info(line)
            self.last_ack_line = line
            self.last_ack_log_time = now

    def reconnect_serial(self):
        try:
            if self.serial_conn is not None:
                self.serial_conn.close()
        except Exception:
            pass

        self.serial_conn = None
        self.last_sent_cmd = None
        self.last_sent_time = 0.0
        self.connect_serial()


def main(args=None):
    rclpy.init(args=args)
    node = SerialEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.serial_conn is not None:
                node.serial_conn.write(b's')
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
