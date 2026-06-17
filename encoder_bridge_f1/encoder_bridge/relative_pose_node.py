#!/usr/bin/env python3
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

        self.multi_aruco_sub = self.create_subscription(
            String,
            '/aruco_multi_markers',
            self.multi_aruco_callback,
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

        self.calib_path = os.path.expanduser(
            '~/robot_ws/src/encoder_bridge/config/camera_calibration.yaml'
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
            'z': 0.0,
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

        self.marker_size_m = 0.085

        self.camera_matrix = None
        self.dist_coeffs = None

        self.load_config()
        self.load_camera_calibration()

        self.x = self.start_pose['x']
        self.y = self.start_pose['y']
        self.yaw = self.start_pose['yaw']

        self.last_marker_id = -1
        self.marker_seen = False
        self.last_source = 'start'

        self.marker_lost_count = 0
        self.marker_lost_limit = 20

        self.current_cmd = 's'
        self.last_left_delta = 0
        self.last_right_delta = 0

        self.wheel_circumference = 0.21
        self.counts_per_turn = 3600.0
        self.wheel_base = 0.23

        self.prev_left_count = None
        self.prev_right_count = None

        self.max_encoder_delta_per_msg = 1800
        self.max_center_distance_per_msg = 0.12
        self.max_delta_yaw_per_msg = math.radians(35.0)

        self.encoder_jump_count = 0

        self.last_pnp_rvec = None
        self.last_pnp_tvec = None
        self.pose_initialized_by_aruco = False

        self.init_candidate = None
        self.init_count = 0
        self.init_required_count = 5
        self.init_candidate_dist_m = 0.06
        self.init_candidate_yaw_deg = 8.0

        self.relocalize_candidate = None
        self.relocalize_count = 0
        self.relocalize_required_count = 5
        self.relocalize_big_jump_m = 0.60
        self.relocalize_candidate_dist_m = 0.12
        self.relocalize_candidate_yaw_deg = 12.0
        self.relocalize_min_marker_count = 3

        self.marker_consistency_filter_enabled = True
        self.marker_consistency_min_count = 3
        self.marker_consistency_dist_m = 0.45
        self.marker_consistency_yaw_deg = 45.0

        self.reprojection_error_warn_px = 15.0
        self.reprojection_error_reject_px = 35.0

        self.single_rb_enabled = True
        self.single_rb_reprojection_error_px = 8.0
        self.single_rb_max_range_m = 1.50
        self.single_rb_good_error_m = 0.15
        self.single_rb_mid_error_m = 0.30
        self.single_rb_reject_error_m = 0.45
        self.single_rb_good_alpha = 0.10
        self.single_rb_mid_alpha = 0.035
        self.single_rb_yaw_alpha = 0.0

        self.get_logger().info('relative_pose_node started.')
        self.get_logger().info('Mode: multi-marker solvePnP + single-marker range-bearing update + stable init + safe relocalization')
        self.get_logger().info(f'Config path: {self.config_path}')
        self.get_logger().info(f'Calibration path: {self.calib_path}')
        self.get_logger().info(f'start_pose: {self.start_pose}')
        self.get_logger().info(f'loaded marker count: {len(self.aruco_marker_pose)}')
        self.get_logger().info(f'camera_mount: {self.camera_mount}')
        self.get_logger().info(f'correction: {self.correction}')
        self.get_logger().info(f'aruco_transform: {self.aruco_transform}')
        self.get_logger().info(f'camera_matrix:\n{self.camera_matrix}')
        self.get_logger().info(f'dist_coeffs: {self.dist_coeffs.ravel()}')
        self.get_logger().info(f'marker_lost_limit: {self.marker_lost_limit}')
        self.get_logger().info(f'init_required_count: {self.init_required_count}')
        self.get_logger().info(f'relocalize_required_count: {self.relocalize_required_count}')
        self.get_logger().info(f'relocalize_big_jump_m: {self.relocalize_big_jump_m}')
        self.get_logger().info(f'relocalize_min_marker_count: {self.relocalize_min_marker_count}')

        if 10 in self.aruco_marker_pose:
            p = self.aruco_marker_pose[10]
            self.get_logger().info(
                f'Marker 10 loaded: x={p["x"]:.3f}, y={p["y"]:.3f}, yaw={p["yaw"]:.4f}'
            )
        else:
            self.get_logger().warn('Marker 10 NOT loaded.')

        if 14 in self.aruco_marker_pose:
            p = self.aruco_marker_pose[14]
            self.get_logger().info(
                f'Marker 14 loaded: x={p["x"]:.3f}, y={p["y"]:.3f}, yaw={p["yaw"]:.4f}'
            )
        else:
            self.get_logger().warn('Marker 14 NOT loaded.')

        if 21 in self.aruco_marker_pose:
            p = self.aruco_marker_pose[21]
            self.get_logger().info(
                f'Marker 21 loaded: x={p["x"]:.3f}, y={p["y"]:.3f}, yaw={p["yaw"]:.4f}'
            )
        else:
            self.get_logger().warn('Marker 21 NOT loaded.')

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
            self.start_pose['x'] = float(data['start_pose'].get('x', 0.0))
            self.start_pose['y'] = float(data['start_pose'].get('y', 0.0))
            self.start_pose['yaw'] = float(data['start_pose'].get('yaw', 0.0))

        if 'aruco_marker_pose' in data:
            self.aruco_marker_pose = {}

            for marker_id, pose in data['aruco_marker_pose'].items():
                try:
                    marker_id_int = int(marker_id)

                    self.aruco_marker_pose[marker_id_int] = {
                        'x': float(pose.get('x', 0.0)),
                        'y': float(pose.get('y', 0.0)),
                        'z': float(pose.get('z', 0.180)),
                        'yaw': float(pose.get('yaw', 0.0))
                    }
                except Exception as e:
                    self.get_logger().warn(
                        f'Invalid marker pose ignored: id={marker_id}, err={e}'
                    )

        if 'camera_mount' in data:
            self.camera_mount['x'] = float(data['camera_mount'].get('x', 0.0))
            self.camera_mount['y'] = float(data['camera_mount'].get('y', 0.0))
            self.camera_mount['z'] = float(data['camera_mount'].get('z', 0.166))
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

    def load_camera_calibration(self):
        if not os.path.exists(self.calib_path):
            raise RuntimeError(f'Calibration file not found: {self.calib_path}')

        with open(self.calib_path, 'r') as f:
            data = yaml.safe_load(f)

        camera_data = data['camera_matrix']['data']
        dist_data = data['distortion_coefficients']['data']

        self.camera_matrix = np.array(camera_data, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(dist_data, dtype=np.float64).reshape(1, -1)

    def multi_aruco_callback(self, msg):
        data = msg.data.strip()
        parsed = self.parse_key_value_message(data, 'MULTI_ARUCO')

        if parsed is None:
            return

        detected = int(parsed.get('detected', 0))
        count = int(parsed.get('count', 0))

        if detected == 0 or count <= 0:
            self.handle_marker_lost()
            return

        marker_blocks = []

        for i in range(count):
            id_key = f'm{i}_id'

            if id_key not in parsed:
                continue

            marker_id = int(parsed[id_key])

            if marker_id not in self.aruco_marker_pose:
                self.get_logger().warn(
                    f'Marker {marker_id} detected but not found in YAML. Skip.'
                )
                continue

            marker_pose = self.aruco_marker_pose[marker_id]
            world_corners = self.get_marker_world_corners(marker_pose)

            valid = True
            img_corners = []

            for corner_idx in range(4):
                x_key = f'm{i}_c{corner_idx}x'
                y_key = f'm{i}_c{corner_idx}y'

                if x_key not in parsed or y_key not in parsed:
                    valid = False
                    break

                img_corners.append([
                    float(parsed[x_key]),
                    float(parsed[y_key])
                ])

            if not valid:
                continue

            marker_blocks.append({
                'id': marker_id,
                'pose': marker_pose,
                'world_corners': world_corners,
                'img_corners': img_corners
            })

        if len(marker_blocks) <= 0:
            self.handle_marker_lost()
            return

        filtered_blocks = self.filter_marker_blocks_by_consistency(marker_blocks)

        object_points = []
        image_points = []
        used_marker_ids = []

        for block in filtered_blocks:
            world_corners = block['world_corners']
            img_corners = block['img_corners']

            for j in range(4):
                object_points.append(world_corners[j])
                image_points.append(img_corners[j])

            used_marker_ids.append(block['id'])

        marker_count = len(used_marker_ids)

        if marker_count <= 0 or len(object_points) < 4:
            self.handle_marker_lost()
            return

        object_points = np.array(object_points, dtype=np.float64)
        image_points = np.array(image_points, dtype=np.float64)

        success, rvec, tvec, reproj_error = self.solve_pnp_best(
            object_points,
            image_points,
            marker_count
        )

        if not success:
            self.get_logger().warn('solvePnP failed.')
            return

        if reproj_error > self.reprojection_error_reject_px:
            self.get_logger().warn(
                f'solvePnP rejected by high reprojection error: '
                f'{reproj_error:.2f}px, markers={used_marker_ids}'
            )
            return

        if reproj_error > self.reprojection_error_warn_px:
            self.get_logger().warn(
                f'solvePnP high reprojection error: '
                f'{reproj_error:.2f}px, markers={used_marker_ids}'
            )

        result = self.compute_robot_pose_from_map_pnp(rvec, tvec)

        robot_x = result['robot_x'] + self.correction['x_offset']
        robot_y = result['robot_y'] + self.correction['y_offset']
        robot_yaw = self.normalize_angle(
            result['robot_yaw'] + self.correction['yaw_offset']
        )

        main_marker_id = used_marker_ids[0] if len(used_marker_ids) > 0 else -1
        main_marker_pose = self.aruco_marker_pose.get(main_marker_id, None)

        single_rb_valid = False
        single_rb_result = None
        single_rb_pos_error = 0.0
        single_rb_range = 0.0
        single_rb_bearing_deg = 0.0
        pose_mode_cmd = f'multi:{marker_count}'

        if marker_count == 1 and len(filtered_blocks) == 1 and self.pose_initialized_by_aruco:
            single_rb_result = self.compute_single_marker_range_bearing_pose(filtered_blocks[0])

            if single_rb_result is not None:
                rb_x = single_rb_result['robot_x'] + self.correction['x_offset']
                rb_y = single_rb_result['robot_y'] + self.correction['y_offset']
                rb_yaw = self.yaw

                single_rb_pos_error = math.hypot(rb_x - self.x, rb_y - self.y)
                single_rb_range = single_rb_result['range_m']
                single_rb_bearing_deg = single_rb_result['bearing_deg']

                if (
                    self.single_rb_enabled
                    and single_rb_result['reproj_error'] <= self.single_rb_reprojection_error_px
                    and single_rb_result['range_m'] <= self.single_rb_max_range_m
                    and single_rb_pos_error <= self.single_rb_reject_error_m
                ):
                    robot_x = rb_x
                    robot_y = rb_y
                    robot_yaw = rb_yaw
                    single_rb_valid = True
                    pose_mode_cmd = 'single_rb:1'
                else:
                    pose_mode_cmd = 'single_pnp_rejected:1'
                    self.get_logger().warn(
                        f'Single RB rejected: marker={main_marker_id}, '
                        f'range={single_rb_range:.3f}, '
                        f'bearing={single_rb_bearing_deg:.1f}, '
                        f'pos_error={single_rb_pos_error:.3f}, '
                        f'reproj={single_rb_result["reproj_error"]:.2f}'
                    )

        self.apply_aruco_pose_with_confidence(
            robot_x,
            robot_y,
            robot_yaw,
            marker_count,
            main_marker_id,
            single_rb_valid=single_rb_valid
        )

        self.last_pnp_rvec = rvec.copy()
        self.last_pnp_tvec = tvec.copy()

        self.marker_lost_count = 0
        self.marker_seen = True

        if main_marker_pose is not None:
            debug_marker_x = main_marker_pose['x']
            debug_marker_y = main_marker_pose['y']
            debug_marker_yaw = main_marker_pose['yaw']
        else:
            debug_marker_x = 0.0
            debug_marker_y = 0.0
            debug_marker_yaw = 0.0

        self.last_marker_id = main_marker_id
        self.last_source = 'multi_marker_solvepnp'

        self.publish_pose(
            cmd=pose_mode_cmd,
            rel_x=result['camera_world_x'],
            rel_y=result['camera_world_y'],
            rel_z=result['camera_world_z'],
            marker_x=debug_marker_x,
            marker_y=debug_marker_y,
            marker_map_yaw=debug_marker_yaw,
            bearing_yaw_deg=single_rb_bearing_deg,
            marker_local_x=result['camera_world_x'],
            marker_local_y=result['camera_world_y'],
            marker_local_z=result['camera_world_z'],
            camera_x=result['camera_world_x'],
            camera_y=result['camera_world_y'],
            robot_x_raw=robot_x,
            robot_y_raw=robot_y,
            robot_yaw_raw=robot_yaw,
            yaw_error_from_rvec=result['robot_yaw'],
            rvec_x=float(rvec[0][0]),
            rvec_y=float(rvec[1][0]),
            rvec_z=float(rvec[2][0]),
            tvec_x=float(tvec[0][0]),
            tvec_y=float(tvec[1][0]),
            tvec_z=float(tvec[2][0]),
            reproj_error=reproj_error
        )

    def handle_marker_lost(self):
        self.marker_lost_count += 1

        if self.marker_lost_count >= self.marker_lost_limit:
            self.marker_seen = False
            self.last_marker_id = -1
            self.init_candidate = None
            self.init_count = 0
            self.relocalize_candidate = None
            self.relocalize_count = 0

    def filter_marker_blocks_by_consistency(self, marker_blocks):
        if not self.marker_consistency_filter_enabled:
            return marker_blocks

        if len(marker_blocks) < self.marker_consistency_min_count:
            return marker_blocks

        candidates = []

        for block in marker_blocks:
            object_points = np.array(block['world_corners'], dtype=np.float64)
            image_points = np.array(block['img_corners'], dtype=np.float64)

            try:
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
            except cv2.error:
                continue

            if not success:
                continue

            result = self.compute_robot_pose_from_map_pnp(rvec, tvec)

            robot_x = result['robot_x'] + self.correction['x_offset']
            robot_y = result['robot_y'] + self.correction['y_offset']
            robot_yaw = self.normalize_angle(
                result['robot_yaw'] + self.correction['yaw_offset']
            )

            candidates.append({
                'id': block['id'],
                'block': block,
                'x': robot_x,
                'y': robot_y,
                'yaw': robot_yaw
            })

        if len(candidates) < self.marker_consistency_min_count:
            return marker_blocks

        median_x = float(np.median([c['x'] for c in candidates]))
        median_y = float(np.median([c['y'] for c in candidates]))
        mean_yaw = self.circular_mean([c['yaw'] for c in candidates])

        inlier_blocks = []
        outlier_ids = []

        for c in candidates:
            dist = math.hypot(c['x'] - median_x, c['y'] - median_y)
            yaw_diff = abs(self.angle_diff(c['yaw'], mean_yaw))

            if (
                dist <= self.marker_consistency_dist_m
                and yaw_diff <= math.radians(self.marker_consistency_yaw_deg)
            ):
                inlier_blocks.append(c['block'])
            else:
                outlier_ids.append(c['id'])

        if len(inlier_blocks) >= 2 and len(outlier_ids) > 0:
            self.get_logger().warn(
                f'Marker consistency filter removed outliers: {outlier_ids}'
            )
            return inlier_blocks

        return marker_blocks

    def circular_mean(self, angles):
        if len(angles) == 0:
            return 0.0

        s = sum(math.sin(a) for a in angles)
        c = sum(math.cos(a) for a in angles)

        return math.atan2(s, c)

    def solve_pnp_best(self, object_points, image_points, marker_count):
        results = []

        if marker_count >= 2 and self.last_pnp_rvec is not None and self.last_pnp_tvec is not None:
            try:
                success_guess, rvec_guess, tvec_guess = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec=self.last_pnp_rvec.copy(),
                    tvec=self.last_pnp_tvec.copy(),
                    useExtrinsicGuess=True,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if success_guess:
                    err_guess = self.compute_reprojection_error(
                        object_points,
                        image_points,
                        rvec_guess,
                        tvec_guess
                    )
                    results.append((success_guess, rvec_guess, tvec_guess, err_guess, 'guess'))
            except cv2.error as e:
                self.get_logger().warn(f'solvePnP with guess failed: {e}')

        try:
            success_fresh, rvec_fresh, tvec_fresh = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success_fresh:
                err_fresh = self.compute_reprojection_error(
                    object_points,
                    image_points,
                    rvec_fresh,
                    tvec_fresh
                )
                results.append((success_fresh, rvec_fresh, tvec_fresh, err_fresh, 'fresh'))
        except cv2.error as e:
            self.get_logger().warn(f'solvePnP fresh failed: {e}')

        if len(results) == 0:
            return False, None, None, 9999.0

        results.sort(key=lambda item: item[3])
        success, rvec, tvec, err, mode = results[0]

        return success, rvec, tvec, err

    def compute_reprojection_error(self, object_points, image_points, rvec, tvec):
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs
        )

        projected = projected.reshape(-1, 2)
        image_points_2d = image_points.reshape(-1, 2)

        errors = np.linalg.norm(projected - image_points_2d, axis=1)

        return float(np.mean(errors))

    def get_marker_world_corners(self, marker_pose):
        cx = marker_pose['x']
        cy = marker_pose['y']
        cz = marker_pose.get('z', 0.180)
        yaw = marker_pose['yaw']

        s = self.marker_size_m / 2.0

        right_x = -math.sin(yaw)
        right_y = math.cos(yaw)
        right_z = 0.0

        up_x = 0.0
        up_y = 0.0
        up_z = 1.0

        center = np.array([cx, cy, cz], dtype=np.float64)
        right = np.array([right_x, right_y, right_z], dtype=np.float64)
        up = np.array([up_x, up_y, up_z], dtype=np.float64)

        top_left = center - right * s + up * s
        top_right = center + right * s + up * s
        bottom_right = center + right * s - up * s
        bottom_left = center - right * s - up * s

        return [
            top_left,
            top_right,
            bottom_right,
            bottom_left
        ]

    def compute_robot_pose_from_map_pnp(self, rvec, tvec):
        rotation_camera_map, _ = cv2.Rodrigues(rvec)

        rotation_map_camera = rotation_camera_map.T
        camera_position_map = -rotation_map_camera @ tvec.reshape(3)

        camera_world_x = float(camera_position_map[0])
        camera_world_y = float(camera_position_map[1])
        camera_world_z = float(camera_position_map[2])

        camera_forward_in_map = rotation_map_camera @ np.array(
            [0.0, 0.0, 1.0],
            dtype=np.float64
        )

        camera_yaw = math.atan2(
            float(camera_forward_in_map[1]),
            float(camera_forward_in_map[0])
        )

        robot_yaw = self.normalize_angle(
            camera_yaw + self.camera_mount['yaw_offset']
        )

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

        robot_x = camera_world_x - offset_map_x
        robot_y = camera_world_y - offset_map_y

        return {
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_yaw': robot_yaw,
            'camera_world_x': camera_world_x,
            'camera_world_y': camera_world_y,
            'camera_world_z': camera_world_z
        }

    def compute_single_marker_range_bearing_pose(self, block):
        marker_pose = block['pose']
        img_corners = np.array(block['img_corners'], dtype=np.float64)

        s = self.marker_size_m / 2.0

        local_object_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float64)

        try:
            success, rvec, tvec = cv2.solvePnP(
                local_object_points,
                img_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        except cv2.error as e:
            self.get_logger().warn(f'Single marker range-bearing solvePnP failed: {e}')
            return None

        if not success:
            return None

        reproj_error = self.compute_reprojection_error(
            local_object_points,
            img_corners,
            rvec,
            tvec
        )

        tx = float(tvec[0][0])
        ty = float(tvec[1][0])
        tz = float(tvec[2][0])

        if tz <= 0.05:
            return None

        range_m = math.sqrt(tx * tx + tz * tz)
        bearing_right = math.atan2(tx, tz)

        robot_yaw_now = self.yaw
        camera_yaw_now = self.normalize_angle(
            robot_yaw_now - self.camera_mount['yaw_offset']
        )

        marker_x = marker_pose['x']
        marker_y = marker_pose['y']

        marker_direction_map = self.normalize_angle(camera_yaw_now - bearing_right)

        camera_x = marker_x - range_m * math.cos(marker_direction_map)
        camera_y = marker_y - range_m * math.sin(marker_direction_map)

        cam_offset_x = self.camera_mount['x']
        cam_offset_y = self.camera_mount['y']

        offset_map_x = (
            cam_offset_x * math.cos(robot_yaw_now)
            - cam_offset_y * math.sin(robot_yaw_now)
        )

        offset_map_y = (
            cam_offset_x * math.sin(robot_yaw_now)
            + cam_offset_y * math.cos(robot_yaw_now)
        )

        robot_x = camera_x - offset_map_x
        robot_y = camera_y - offset_map_y

        return {
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_yaw': robot_yaw_now,
            'camera_x': camera_x,
            'camera_y': camera_y,
            'range_m': range_m,
            'bearing_rad': bearing_right,
            'bearing_deg': math.degrees(bearing_right),
            'reproj_error': reproj_error,
            'rvec': rvec,
            'tvec': tvec,
            'tx': tx,
            'ty': ty,
            'tz': tz
        }

    def get_marker_count_alpha(self, marker_count):
        if marker_count >= 5:
            return 0.45, 0.45
        if marker_count == 4:
            return 0.40, 0.40
        if marker_count == 3:
            return 0.30, 0.35
        if marker_count == 2:
            return 0.08, 0.22
        if marker_count == 1:
            return 0.00, 0.00

        return 0.00, 0.00

    def apply_aruco_pose_with_confidence(self, robot_x, robot_y, robot_yaw, marker_count, main_marker_id, single_rb_valid=False):
        pos_alpha, yaw_alpha = self.get_marker_count_alpha(marker_count)

        if self.aruco_transform['use_smoothing']:
            config_alpha = self.aruco_transform['alpha']
            pos_alpha = min(pos_alpha, config_alpha)
            yaw_alpha = min(yaw_alpha, config_alpha)
        else:
            pos_alpha = min(pos_alpha, 1.0)
            yaw_alpha = min(yaw_alpha, 1.0)

        if not self.pose_initialized_by_aruco:
            if marker_count >= 2:
                self.x = robot_x
                self.y = robot_y
                self.yaw = robot_yaw
                self.pose_initialized_by_aruco = True
                self.init_candidate = None
                self.init_count = 0
                self.relocalize_candidate = None
                self.relocalize_count = 0

                self.get_logger().warn(
                    f'Initial ArUco pose set by multi marker: '
                    f'x={self.x:.3f}, y={self.y:.3f}, '
                    f'yaw={math.degrees(self.yaw):.1f}, markers={marker_count}'
                )
            else:
                self.try_single_marker_initialization(
                    robot_x,
                    robot_y,
                    robot_yaw,
                    main_marker_id
                )

            return

        pos_error = math.hypot(robot_x - self.x, robot_y - self.y)
        yaw_error = abs(self.angle_diff(robot_yaw, self.yaw))

        if (
            pos_error > self.relocalize_big_jump_m
            and marker_count >= self.relocalize_min_marker_count
        ):
            self.try_relocalize(robot_x, robot_y, robot_yaw, marker_count, pos_error)
            return

        self.relocalize_candidate = None
        self.relocalize_count = 0

        if marker_count == 1:
            yaw_alpha = self.single_rb_yaw_alpha

            if single_rb_valid:
                if pos_error < self.single_rb_good_error_m:
                    pos_alpha = self.single_rb_good_alpha
                elif pos_error < self.single_rb_mid_error_m:
                    pos_alpha = self.single_rb_mid_alpha
                else:
                    pos_alpha = 0.0
            else:
                pos_alpha = 0.0

        if marker_count == 2:
            if yaw_error > math.radians(10.0):
                pos_alpha = min(pos_alpha, 0.03)

            if pos_error > 0.45:
                pos_alpha = 0.0
            elif pos_error > 0.25:
                pos_alpha = min(pos_alpha, 0.03)

        if marker_count >= 3:
            if pos_error > 0.60:
                pos_alpha = 0.0
            elif pos_error > 0.35:
                pos_alpha = min(pos_alpha, 0.08)

        if yaw_error > math.radians(45.0) and marker_count <= 2:
            yaw_alpha = min(yaw_alpha, 0.05)

        self.x = (1.0 - pos_alpha) * self.x + pos_alpha * robot_x
        self.y = (1.0 - pos_alpha) * self.y + pos_alpha * robot_y
        self.yaw = self.smooth_angle(self.yaw, robot_yaw, yaw_alpha)

    def try_single_marker_initialization(self, robot_x, robot_y, robot_yaw, marker_id):
        if marker_id < 0:
            return

        if self.init_candidate is None:
            self.init_candidate = {
                'x': robot_x,
                'y': robot_y,
                'yaw': robot_yaw,
                'marker_id': marker_id
            }
            self.init_count = 1
        else:
            same_marker = int(self.init_candidate['marker_id']) == int(marker_id)

            candidate_dist = math.hypot(
                robot_x - self.init_candidate['x'],
                robot_y - self.init_candidate['y']
            )

            candidate_yaw_diff = abs(
                self.angle_diff(robot_yaw, self.init_candidate['yaw'])
            )

            if (
                same_marker
                and candidate_dist < self.init_candidate_dist_m
                and candidate_yaw_diff < math.radians(self.init_candidate_yaw_deg)
            ):
                self.init_count += 1
            else:
                self.init_candidate = {
                    'x': robot_x,
                    'y': robot_y,
                    'yaw': robot_yaw,
                    'marker_id': marker_id
                }
                self.init_count = 1

        self.get_logger().warn(
            f'Single marker init candidate '
            f'{self.init_count}/{self.init_required_count}: '
            f'marker={marker_id}, '
            f'raw=({robot_x:.3f},{robot_y:.3f},{math.degrees(robot_yaw):.1f})'
        )

        if self.init_count >= self.init_required_count:
            self.x = robot_x
            self.y = robot_y
            self.yaw = robot_yaw
            self.pose_initialized_by_aruco = True
            self.init_candidate = None
            self.init_count = 0

            self.get_logger().warn(
                f'Initial ArUco pose set by stable single marker: '
                f'x={self.x:.3f}, y={self.y:.3f}, '
                f'yaw={math.degrees(self.yaw):.1f}, marker={marker_id}'
            )

    def try_relocalize(self, robot_x, robot_y, robot_yaw, marker_count, pos_error):
        if self.relocalize_candidate is None:
            self.relocalize_candidate = {
                'x': robot_x,
                'y': robot_y,
                'yaw': robot_yaw
            }
            self.relocalize_count = 1
        else:
            candidate_dist = math.hypot(
                robot_x - self.relocalize_candidate['x'],
                robot_y - self.relocalize_candidate['y']
            )

            candidate_yaw_diff = abs(
                self.angle_diff(robot_yaw, self.relocalize_candidate['yaw'])
            )

            if (
                candidate_dist < self.relocalize_candidate_dist_m
                and candidate_yaw_diff < math.radians(self.relocalize_candidate_yaw_deg)
            ):
                self.relocalize_count += 1
            else:
                self.relocalize_candidate = {
                    'x': robot_x,
                    'y': robot_y,
                    'yaw': robot_yaw
                }
                self.relocalize_count = 1

        self.get_logger().warn(
            f'Relocalize candidate '
            f'{self.relocalize_count}/{self.relocalize_required_count}: '
            f'raw=({robot_x:.3f},{robot_y:.3f},{math.degrees(robot_yaw):.1f}), '
            f'current=({self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f}), '
            f'pos_error={pos_error:.3f}, markers={marker_count}'
        )

        if self.relocalize_count >= self.relocalize_required_count:
            self.x = robot_x
            self.y = robot_y
            self.yaw = robot_yaw
            self.relocalize_candidate = None
            self.relocalize_count = 0

            self.get_logger().warn(
                f'FORCE RELOCALIZED by stable ArUco: '
                f'x={self.x:.3f}, y={self.y:.3f}, '
                f'yaw={math.degrees(self.yaw):.1f}, markers={marker_count}'
            )

    def encoder_callback(self, msg):
        data = msg.data.strip()
        parsed = self.parse_encoder_message(data)

        if parsed is None:
            return

        left_count = parsed['left_count']
        right_count = parsed['right_count']
        arduino_left_delta = parsed['left_delta']
        arduino_right_delta = parsed['right_delta']
        cmd = parsed['cmd']

        self.current_cmd = cmd

        if cmd == 'c':
            self.prev_left_count = left_count
            self.prev_right_count = right_count
            self.last_left_delta = 0
            self.last_right_delta = 0
            self.last_source = 'encoder_clear'
            self.init_candidate = None
            self.init_count = 0
            self.relocalize_candidate = None
            self.relocalize_count = 0

            self.publish_pose(
                left_count=left_count,
                right_count=right_count,
                left_delta=0,
                right_delta=0,
                cmd=cmd
            )
            return

        if self.prev_left_count is None or self.prev_right_count is None:
            self.prev_left_count = left_count
            self.prev_right_count = right_count
            self.last_left_delta = 0
            self.last_right_delta = 0

            self.publish_pose(
                left_count=left_count,
                right_count=right_count,
                left_delta=0,
                right_delta=0,
                cmd=cmd
            )
            return

        node_left_delta = left_count - self.prev_left_count
        node_right_delta = right_count - self.prev_right_count

        self.prev_left_count = left_count
        self.prev_right_count = right_count

        left_delta = node_left_delta
        right_delta = node_right_delta

        if abs(left_delta) > self.max_encoder_delta_per_msg or abs(right_delta) > self.max_encoder_delta_per_msg:
            self.encoder_jump_count += 1
            self.get_logger().warn(
                f'Encoder jump ignored. '
                f'node_delta=({left_delta},{right_delta}), '
                f'arduino_delta=({arduino_left_delta},{arduino_right_delta}), '
                f'count=({left_count},{right_count}), '
                f'cmd={cmd}, '
                f'jump_count={self.encoder_jump_count}'
            )

            self.last_left_delta = 0
            self.last_right_delta = 0

            self.publish_pose(
                left_count=left_count,
                right_count=right_count,
                left_delta=0,
                right_delta=0,
                cmd=f'{cmd}:encoder_jump_ignored'
            )
            return

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

        if abs(center_distance) > self.max_center_distance_per_msg:
            self.encoder_jump_count += 1
            self.get_logger().warn(
                f'Encoder center distance too large. Ignored. '
                f'center_distance={center_distance:.4f}, '
                f'left_distance={left_distance:.4f}, '
                f'right_distance={right_distance:.4f}, '
                f'left_delta={left_delta}, '
                f'right_delta={right_delta}, '
                f'cmd={cmd}'
            )

            self.publish_pose(
                left_count=left_count,
                right_count=right_count,
                left_delta=left_delta,
                right_delta=right_delta,
                cmd=f'{cmd}:center_jump_ignored',
                left_distance=left_distance,
                right_distance=right_distance,
                center_distance=0.0,
                delta_yaw=0.0
            )
            return

        if abs(delta_yaw) > self.max_delta_yaw_per_msg:
            self.encoder_jump_count += 1
            clipped_delta_yaw = max(
                -self.max_delta_yaw_per_msg,
                min(self.max_delta_yaw_per_msg, delta_yaw)
            )

            self.get_logger().warn(
                f'Encoder delta_yaw clipped. '
                f'raw_delta_yaw_deg={math.degrees(delta_yaw):.2f}, '
                f'clipped_delta_yaw_deg={math.degrees(clipped_delta_yaw):.2f}, '
                f'left_delta={left_delta}, '
                f'right_delta={right_delta}, '
                f'cmd={cmd}'
            )

            delta_yaw = clipped_delta_yaw

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
        delta_yaw=0.0,
        reproj_error=0.0
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
            f'reproj_error={reproj_error:.2f},'
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
