import math
import os
import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RelativePoseNode(Node):
    def __init__(self):
        super().__init__('relative_pose_node')

        self.aruco_sub = self.create_subscription(
            String,
            '/aruco_marker',
            self.aruco_callback,
            10
        )

        self.encoder_sub = self.create_subscription(
            String,
            '/encoder_counts',
            self.encoder_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            String,
            '/relative_pose',
            10
        )

        self.config_path = os.path.expanduser(
            '~/robot_ws/src/encoder_bridge/config/aruco_reference.yaml'
        )

        self.start_pose = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0
        }

        self.aruco_marker_pose = {}

        self.camera_mount = {
            'x': 0.0,
            'y': 0.0,
            'yaw_offset': 0.0
        }

        self.correction = {
            'x_offset': 0.0,
            'y_offset': 0.0,
            'yaw_offset': 0.0
        }

        self.aruco_transform = {
            'forward_sign': 1.0,
            'lateral_sign': 1.0,
            'yaw_sign': 1.0,
            'use_smoothing': True,
            'alpha': 0.75
        }

        self.load_config()

        self.x = self.start_pose['x']
        self.y = self.start_pose['y']
        self.yaw = self.start_pose['yaw']

        self.last_marker_id = -1
        self.marker_seen = False
        self.last_source = 'start'

        self.marker_lost_count = 0
        self.marker_lost_limit = 5

        self.current_cmd = 's'
        self.last_left_delta = 0
        self.last_right_delta = 0

        self.wheel_circumference = 0.21
        self.counts_per_turn = 3600.0
        self.wheel_base = 0.23

        self.get_logger().info('relative_pose_node started.')
        self.get_logger().info('Mode: ArUco priority + stable tvec position + encoder backup')
        self.get_logger().info(f'Config path: {self.config_path}')
        self.get_logger().info(f'start_pose: {self.start_pose}')
        self.get_logger().info(f'aruco_marker_pose: {self.aruco_marker_pose}')
        self.get_logger().info(f'camera_mount: {self.camera_mount}')
        self.get_logger().info(f'correction: {self.correction}')
        self.get_logger().info(f'aruco_transform: {self.aruco_transform}')
        self.get_logger().info(f'wheel_base: {self.wheel_base}')

        self.publish_pose(cmd='init')

    def load_config(self):
        if not os.path.exists(self.config_path):
            self.get_logger().warn(f'Config file not found: {self.config_path}')
            return

        with open(self.config_path, 'r') as f:
            data = yaml.safe_load(f)

        if data is None:
            self.get_logger().warn('Config file is empty.')
            return

        if 'start_pose' in data:
            self.start_pose['x'] = float(data['start_pose']['x'])
            self.start_pose['y'] = float(data['start_pose']['y'])
            self.start_pose['yaw'] = float(data['start_pose']['yaw'])

        if 'aruco_marker_pose' in data:
            for marker_id, pose in data['aruco_marker_pose'].items():
                marker_id_int = int(marker_id)

                self.aruco_marker_pose[marker_id_int] = {
                    'x': float(pose['x']),
                    'y': float(pose['y']),
                    'yaw': float(pose['yaw'])
                }

        if 'camera_mount' in data:
            self.camera_mount['x'] = float(data['camera_mount'].get('x', 0.0))
            self.camera_mount['y'] = float(data['camera_mount'].get('y', 0.0))
            self.camera_mount['yaw_offset'] = float(
                data['camera_mount'].get('yaw_offset', 0.0)
            )

        if 'correction' in data:
            self.correction['x_offset'] = float(
                data['correction'].get('x_offset', 0.0)
            )
            self.correction['y_offset'] = float(
                data['correction'].get('y_offset', 0.0)
            )
            self.correction['yaw_offset'] = float(
                data['correction'].get('yaw_offset', 0.0)
            )

        if 'aruco_transform' in data:
            self.aruco_transform['forward_sign'] = float(
                data['aruco_transform'].get('forward_sign', 1.0)
            )
            self.aruco_transform['lateral_sign'] = float(
                data['aruco_transform'].get('lateral_sign', 1.0)
            )
            self.aruco_transform['yaw_sign'] = float(
                data['aruco_transform'].get('yaw_sign', 1.0)
            )
            self.aruco_transform['use_smoothing'] = bool(
                data['aruco_transform'].get('use_smoothing', True)
            )
            self.aruco_transform['alpha'] = float(
                data['aruco_transform'].get('alpha', 0.75)
            )

    def aruco_callback(self, msg):
        data = msg.data.strip()
        parsed = self.parse_key_value_message(data, 'ARUCO')

        if parsed is None:
            return

        marker_id = int(parsed.get('id', -1))
        detected = int(parsed.get('detected', 0))

        if detected == 0 or marker_id == -1:
            self.marker_lost_count += 1

            if self.marker_lost_count >= self.marker_lost_limit:
                self.marker_seen = False
                self.last_marker_id = -1

            return

        self.marker_lost_count = 0
        self.marker_seen = True
        self.last_marker_id = marker_id

        if marker_id not in self.aruco_marker_pose:
            self.get_logger().warn(f'Marker ID {marker_id} has no map pose.')
            return

        required_keys = [
            'rvec_x',
            'rvec_y',
            'rvec_z',
            'tvec_x',
            'tvec_y',
            'tvec_z'
        ]

        for key in required_keys:
            if key not in parsed:
                self.get_logger().warn(f'Aruco message has no {key}.')
                return

        rvec = np.array(
            [
                float(parsed['rvec_x']),
                float(parsed['rvec_y']),
                float(parsed['rvec_z'])
            ],
            dtype=np.float64
        )

        tvec = np.array(
            [
                float(parsed['tvec_x']),
                float(parsed['tvec_y']),
                float(parsed['tvec_z'])
            ],
            dtype=np.float64
        )

        rel_x = float(parsed.get('rel_x', tvec[0]))
        rel_y = float(parsed.get('rel_y', tvec[1]))
        rel_z = float(parsed.get('rel_z', tvec[2]))

        bearing_yaw_deg = float(parsed.get('bearing_yaw_deg', 0.0))
        marker_roll = float(parsed.get('marker_roll', 0.0))
        marker_pitch = float(parsed.get('marker_pitch', 0.0))
        marker_yaw = float(parsed.get('marker_yaw', 0.0))

        marker_pose = self.aruco_marker_pose[marker_id]

        result = self.compute_robot_pose_from_rvec_tvec(
            marker_pose,
            rvec,
            tvec
        )

        robot_x = result['robot_x'] + self.correction['x_offset']
        robot_y = result['robot_y'] + self.correction['y_offset']
        robot_yaw = self.normalize_angle(
            result['robot_yaw'] + self.correction['yaw_offset']
        )

        if self.aruco_transform['use_smoothing']:
            alpha = self.aruco_transform['alpha']

            self.x = (1.0 - alpha) * self.x + alpha * robot_x
            self.y = (1.0 - alpha) * self.y + alpha * robot_y
            self.yaw = self.smooth_angle(self.yaw, robot_yaw, alpha)
        else:
            self.x = robot_x
            self.y = robot_y
            self.yaw = robot_yaw

        self.last_source = 'aruco_full_pose'

        self.publish_pose(
            cmd='aruco',
            rel_x=rel_x,
            rel_y=rel_y,
            rel_z=rel_z,
            marker_roll=marker_roll,
            marker_pitch=marker_pitch,
            marker_yaw=marker_yaw,
            bearing_yaw_deg=bearing_yaw_deg,
            rvec_x=rvec[0],
            rvec_y=rvec[1],
            rvec_z=rvec[2],
            tvec_x=tvec[0],
            tvec_y=tvec[1],
            tvec_z=tvec[2],
            marker_x=marker_pose['x'],
            marker_y=marker_pose['y'],
            marker_map_yaw=marker_pose['yaw'],
            marker_local_x=result['marker_local_x'],
            marker_local_y=result['marker_local_y'],
            marker_local_z=result['marker_local_z'],
            camera_x=result['camera_x'],
            camera_y=result['camera_y'],
            robot_x_raw=robot_x,
            robot_y_raw=robot_y,
            robot_yaw_raw=robot_yaw,
            yaw_error_from_rvec=result['yaw_error']
        )

    def compute_robot_pose_from_rvec_tvec(self, marker_pose, rvec, tvec):
        marker_map_yaw = marker_pose['yaw']

        rel_x = float(tvec[0])
        rel_y = float(tvec[1])
        rel_z = float(tvec[2])

        forward = self.aruco_transform['forward_sign'] * rel_z
        lateral = self.aruco_transform['lateral_sign'] * rel_x

        heading_x = math.cos(marker_map_yaw)
        heading_y = math.sin(marker_map_yaw)

        right_x = -math.sin(marker_map_yaw)
        right_y = math.cos(marker_map_yaw)

        camera_x = marker_pose['x'] + forward * heading_x + lateral * right_x
        camera_y = marker_pose['y'] + forward * heading_y + lateral * right_y

        rotation_camera_marker, _ = cv2.Rodrigues(rvec)

        marker_yaw_from_rvec = math.atan2(
            float(rotation_camera_marker[1, 0]),
            float(rotation_camera_marker[0, 0])
        )

        yaw_error = -marker_yaw_from_rvec

        robot_yaw = self.normalize_angle(
            marker_map_yaw
            - math.pi
            + self.aruco_transform['yaw_sign'] * yaw_error
            + self.camera_mount['yaw_offset']
        )

        robot_x, robot_y = self.apply_camera_mount_offset(
            camera_x,
            camera_y,
            robot_yaw
        )

        return {
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_yaw': robot_yaw,
            'camera_x': camera_x,
            'camera_y': camera_y,
            'marker_local_x': rel_x,
            'marker_local_y': rel_y,
            'marker_local_z': rel_z,
            'yaw_error': yaw_error
        }

    def apply_camera_mount_offset(self, camera_x, camera_y, robot_yaw):
        cam_offset_x = self.camera_mount['x']
        cam_offset_y = self.camera_mount['y']

        offset_map_x = (
            cam_offset_x * math.cos(robot_yaw)
            - cam_offset_y * math.sin(robot_yaw)
        )

        offset_map_y = (
            cam_offset_x * math.sin(robot_yaw)
            + cam_offset_y * math.cos(robot_yaw)
        )

        robot_x = camera_x - offset_map_x
        robot_y = camera_y - offset_map_y

        return robot_x, robot_y

    def encoder_callback(self, msg):
        data = msg.data.strip()
        parsed = self.parse_encoder_message(data)

        if parsed is None:
            return

        left_count = parsed['left_count']
        right_count = parsed['right_count']
        left_delta = parsed['left_delta']
        right_delta = parsed['right_delta']
        cmd = parsed['cmd']

        self.current_cmd = cmd
        self.last_left_delta = left_delta
        self.last_right_delta = right_delta

        if self.marker_seen:
            return

        if cmd == 's':
            self.last_source = 'stop'

            self.publish_pose(
                left_count=left_count,
                right_count=right_count,
                left_delta=left_delta,
                right_delta=right_delta,
                cmd=cmd
            )
            return

        if cmd not in ['w', 'a', 'd', 'q', 'e']:
            return

        left_distance = self.left_delta_to_distance(left_delta)
        right_distance = self.right_delta_to_distance(right_delta)

        center_distance = (left_distance + right_distance) / 2.0
        delta_yaw = (right_distance - left_distance) / self.wheel_base

        mid_yaw = self.yaw + delta_yaw / 2.0

        self.x += center_distance * math.cos(mid_yaw)
        self.y += center_distance * math.sin(mid_yaw)
        self.yaw += delta_yaw
        self.yaw = self.normalize_angle(self.yaw)

        if cmd in ['q', 'e']:
            self.last_source = 'encoder_turn'
        else:
            self.last_source = 'encoder_backup'

        self.publish_pose(
            left_count=left_count,
            right_count=right_count,
            left_delta=left_delta,
            right_delta=right_delta,
            cmd=cmd,
            left_distance=left_distance,
            right_distance=right_distance,
            center_distance=center_distance,
            delta_yaw=delta_yaw
        )

    def left_delta_to_distance(self, left_delta):
        turns = (-left_delta) / self.counts_per_turn
        return turns * self.wheel_circumference

    def right_delta_to_distance(self, right_delta):
        turns = right_delta / self.counts_per_turn
        return turns * self.wheel_circumference

    def parse_key_value_message(self, data, prefix):
        if not data.startswith(prefix + ','):
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

    def parse_encoder_message(self, data):
        if not data.startswith('ENC,'):
            return None

        parts = data.split(',')

        if len(parts) != 6:
            return None

        try:
            result = {
                'left_count': int(parts[1]),
                'right_count': int(parts[2]),
                'left_delta': int(parts[3]),
                'right_delta': int(parts[4]),
                'cmd': parts[5].strip()
            }
        except ValueError:
            return None

        return result

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def angle_diff(self, target, current):
        return self.normalize_angle(target - current)

    def smooth_angle(self, current, target, alpha):
        diff = self.angle_diff(target, current)
        return self.normalize_angle(current + alpha * diff)

    def publish_pose(
        self,
        left_count=0,
        right_count=0,
        left_delta=0,
        right_delta=0,
        cmd='none',
        rel_x=0.0,
        rel_y=0.0,
        rel_z=0.0,
        marker_roll=0.0,
        marker_pitch=0.0,
        marker_yaw=0.0,
        bearing_yaw_deg=0.0,
        rvec_x=0.0,
        rvec_y=0.0,
        rvec_z=0.0,
        tvec_x=0.0,
        tvec_y=0.0,
        tvec_z=0.0,
        marker_x=0.0,
        marker_y=0.0,
        marker_map_yaw=0.0,
        marker_local_x=0.0,
        marker_local_y=0.0,
        marker_local_z=0.0,
        camera_x=0.0,
        camera_y=0.0,
        robot_x_raw=0.0,
        robot_y_raw=0.0,
        robot_yaw_raw=0.0,
        yaw_error_from_rvec=0.0,
        left_distance=0.0,
        right_distance=0.0,
        center_distance=0.0,
        delta_yaw=0.0
    ):
        yaw_deg = math.degrees(self.yaw)
        robot_yaw_raw_deg = math.degrees(robot_yaw_raw)
        marker_map_yaw_deg = math.degrees(marker_map_yaw)
        yaw_error_from_rvec_deg = math.degrees(yaw_error_from_rvec)

        delta_yaw_deg = math.degrees(delta_yaw)
        marker_roll_deg = math.degrees(marker_roll)
        marker_pitch_deg = math.degrees(marker_pitch)
        marker_yaw_deg = math.degrees(marker_yaw)

        out = (
            f'RELPOSE,'
            f'x={self.x:.3f},'
            f'y={self.y:.3f},'
            f'yaw={self.yaw:.3f},'
            f'yaw_deg={yaw_deg:.2f},'
            f'source={self.last_source},'
            f'marker_id={self.last_marker_id},'
            f'marker_seen={1 if self.marker_seen else 0},'
            f'rel_x={rel_x:.3f},'
            f'rel_y={rel_y:.3f},'
            f'rel_z={rel_z:.3f},'
            f'bearing_yaw_deg={bearing_yaw_deg:.2f},'
            f'marker_roll={marker_roll:.4f},'
            f'marker_roll_deg={marker_roll_deg:.2f},'
            f'marker_pitch={marker_pitch:.4f},'
            f'marker_pitch_deg={marker_pitch_deg:.2f},'
            f'marker_yaw={marker_yaw:.4f},'
            f'marker_yaw_deg={marker_yaw_deg:.2f},'
            f'rvec_x={rvec_x:.6f},'
            f'rvec_y={rvec_y:.6f},'
            f'rvec_z={rvec_z:.6f},'
            f'tvec_x={tvec_x:.6f},'
            f'tvec_y={tvec_y:.6f},'
            f'tvec_z={tvec_z:.6f},'
            f'marker_x={marker_x:.3f},'
            f'marker_y={marker_y:.3f},'
            f'marker_map_yaw={marker_map_yaw:.4f},'
            f'marker_map_yaw_deg={marker_map_yaw_deg:.2f},'
            f'marker_local_x={marker_local_x:.3f},'
            f'marker_local_y={marker_local_y:.3f},'
            f'marker_local_z={marker_local_z:.3f},'
            f'camera_x={camera_x:.3f},'
            f'camera_y={camera_y:.3f},'
            f'robot_x_raw={robot_x_raw:.3f},'
            f'robot_y_raw={robot_y_raw:.3f},'
            f'robot_yaw_raw={robot_yaw_raw:.4f},'
            f'robot_yaw_raw_deg={robot_yaw_raw_deg:.2f},'
            f'yaw_error_from_rvec={yaw_error_from_rvec:.4f},'
            f'yaw_error_from_rvec_deg={yaw_error_from_rvec_deg:.2f},'
            f'left_count={left_count},'
            f'right_count={right_count},'
            f'left_delta={left_delta},'
            f'right_delta={right_delta},'
            f'left_dist={left_distance:.4f},'
            f'right_dist={right_distance:.4f},'
            f'center_dist={center_distance:.4f},'
            f'delta_yaw={delta_yaw:.4f},'
            f'delta_yaw_deg={delta_yaw_deg:.2f},'
            f'cmd={cmd}'
        )

        msg = String()
        msg.data = out

        self.pose_pub.publish(msg)
        self.get_logger().info(out)


def main(args=None):
    rclpy.init(args=args)

    node = RelativePoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
