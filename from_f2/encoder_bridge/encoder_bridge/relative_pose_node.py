#!/usr/bin/env python3
import math
import os
from collections import deque
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

        self.marker_size_m = 0.050

        self.camera_matrix = None
        self.dist_coeffs = None

        self.load_config()
        self.load_camera_calibration()

        self.x = self.start_pose['x']
        self.y = self.start_pose['y']
        self.yaw = self.start_pose['yaw']

        # Raw ArUco pose and robust control-pose filter state.
        # The filter does not reject movement by x/y direction. It uses only
        # temporal consistency, total 2-D displacement, yaw change, marker
        # count, and reprojection quality.
        self.raw_x = self.x
        self.raw_y = self.y
        self.raw_yaw = self.yaw

        self.raw_pose_buffer = deque(maxlen=5)
        self.robust_pose_min_samples = 3

        self.max_normal_step_m = 0.25
        self.max_normal_yaw_step_deg = 25.0

        self.control_jump_candidate = None
        self.control_jump_candidate_count = 0
        self.control_jump_required_3plus = 10
        self.control_jump_required_2 = 15
        self.control_jump_candidate_dist_m = 0.08
        self.control_jump_candidate_yaw_deg = 8.0
        self.rejected_pose_clear_buffer = False
        self.allow_relocalize_only_when_stopped = True

        # Stationary recovery is used only when the robot is stopped and
        # ArUco gives a stable multi-marker pose. It is intentionally slow:
        # it prevents the map robot dot from jumping while still allowing
        # recovery when the stored pose is wrong.
        self.stationary_recovery_candidate = None
        self.stationary_recovery_count = 0
        self.stationary_recovery_required_count = 2
        self.stationary_recovery_min_markers = 2
        self.stationary_recovery_max_jump_m = 0.55
        self.stationary_recovery_max_yaw_deg = 70.0
        self.stationary_recovery_candidate_dist_m = 0.180
        self.stationary_recovery_candidate_yaw_deg = 18.0
        self.stationary_recovery_max_spread_m = 0.180
        self.stationary_recovery_max_yaw_spread_deg = 16.0
        self.stationary_recovery_max_reproj_px = 4.0
        self.stationary_recovery_pos_alpha = 0.45
        self.stationary_recovery_yaw_alpha = 0.85

        # STOP 상태 강제 재위치화.
        # 로봇이 멈춰 있고 여러 마커가 보이면 기존 pose가 틀렸을 가능성이 크므로
        # 기존 pose 기준 jump를 거의 믿지 않고 ArUco 후보를 우선한다.
        self.stationary_force_candidate = None
        self.stationary_force_count = 0
        self.stationary_force_required_count = 3
        self.stationary_force_min_markers = 2
        self.stationary_force_prefer_markers = 4
        self.stationary_force_max_jump_m = 1.20
        self.stationary_force_max_yaw_deg = 180.0
        self.stationary_force_max_reproj_px = 4.20
        self.stationary_force_max_reproj_px_2marker = 3.60
        self.stationary_force_candidate_dist_m = 0.60
        self.stationary_force_candidate_yaw_deg = 75.0
        self.stationary_force_max_spread_m = 0.60
        self.stationary_force_max_yaw_spread_deg = 80.0
        self.stationary_force_pos_alpha = 1.00
        self.stationary_force_yaw_alpha = 1.00
        self.stationary_force_map_margin_m = 1.00

        self.stable_two_marker_candidate = None
        self.stable_two_marker_count = 0
        self.stable_two_marker_required_stopped = 2
        self.stable_two_marker_required_moving = 2
        self.stable_two_marker_candidate_dist_m = 0.250
        self.stable_two_marker_candidate_yaw_deg = 30.0
        self.stable_two_marker_max_jump_m = 0.85
        self.stable_two_marker_max_yaw_deg = 80.0
        self.stable_two_marker_max_reproj_px = 4.0
        self.stable_two_marker_pos_alpha_stopped = 0.45
        self.stable_two_marker_yaw_alpha_stopped = 0.85
        self.stable_two_marker_pos_alpha_moving = 0.35
        self.stable_two_marker_yaw_alpha_moving = 0.50

        self.moving_high_conf_min_markers = 2
        self.moving_high_conf_max_reproj_px = 3.5
        self.moving_high_conf_max_raw_jump_m = 0.85
        self.moving_high_conf_max_raw_yaw_deg = 75.0
        self.moving_high_conf_max_robust_jump_m = 0.75
        self.moving_high_conf_max_robust_yaw_deg = 75.0
        self.moving_high_conf_pos_alpha = 0.42
        self.moving_high_conf_yaw_alpha = 0.45

        # x/y는 거의 맞는데 yaw만 30도 이상 튀는 현장 문제 대응.
        # markers가 충분하고 reprojection/spread가 괜찮으면 yaw만 크다고 버리지 않는다.
        self.close_position_yaw_recovery_min_markers = 4
        self.close_position_yaw_recovery_max_dist_m = 0.12
        self.close_position_yaw_recovery_max_yaw_deg = 60.0
        self.close_position_yaw_recovery_max_reproj_px = 2.5
        self.close_position_yaw_recovery_max_spread_m = 0.18
        self.close_position_yaw_recovery_max_yaw_spread_deg = 14.0
        self.close_position_yaw_recovery_pos_alpha_stopped = 0.05
        self.close_position_yaw_recovery_yaw_alpha_stopped = 0.12
        self.close_position_yaw_recovery_pos_alpha_moving = 0.10
        self.close_position_yaw_recovery_yaw_alpha_moving = 0.30

        self.single_marker_recovery_candidate = None
        self.single_marker_recovery_count = 0
        self.single_marker_recovery_required_stopped = 2
        self.single_marker_recovery_required_moving = 2
        self.single_marker_recovery_candidate_dist_m = 0.12
        self.single_marker_recovery_candidate_yaw_deg = 8.0
        self.single_marker_recovery_max_jump_m = 0.70
        self.single_marker_recovery_max_yaw_deg = 12.0
        self.single_marker_recovery_max_reproj_px = 8.0
        self.single_marker_recovery_max_range_m = 1.80
        self.single_marker_recovery_pos_alpha_stopped = 0.85
        self.single_marker_recovery_yaw_alpha_stopped = 0.20
        self.single_marker_recovery_pos_alpha_moving = 0.35
        self.single_marker_recovery_yaw_alpha_moving = 0.05

        self.last_robust_x = self.x
        self.last_robust_y = self.y
        self.last_robust_yaw = self.yaw

        self.last_marker_id = -1
        self.marker_seen = False
        self.aruco_pose_accepted = False
        self.last_source = 'start'
        self.last_selected_hyp_label = ''
        self.last_selected_ids_text = ''
        self.last_candidate_count = 0
        self.last_active_face_key_text = ''

        self.marker_lost_count = 0
        self.marker_lost_limit = 20

        self.pose_log_enabled = False
        self.pose_log_interval_sec = 2.0
        self.last_pose_log_time = None
        self.last_pose_log_key = None

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
        self.max_delta_yaw_per_msg = math.radians(45.0)

        self.encoder_jump_count = 0

        self.last_pnp_rvec = None
        self.last_pnp_tvec = None
        self.pose_initialized_by_aruco = False

        # BOOT_INIT: starting pose must be stable before it is accepted.
        # Initial pose can now be accepted with 2 markers if the candidate is
        # very stable for several frames. 1 marker is still not accepted for
        # initial pose because single-marker yaw can create wrong starts.
        self.boot_init_min_markers = 2
        self.boot_init_max_reproj_px = 2.2
        self.boot_init_max_position_spread_m = 0.16
        self.boot_init_max_yaw_spread_deg = 8.0
        self.boot_init_max_position_spread_abs_m = 0.24
        self.boot_init_max_yaw_spread_abs_deg = 12.0

        self.boot_init_2marker_max_reproj_px = 2.0
        self.boot_init_2marker_max_position_spread_m = 0.09
        self.boot_init_2marker_max_position_spread_abs_m = 0.14
        self.boot_init_2marker_max_yaw_spread_deg = 6.0
        self.boot_init_2marker_max_yaw_spread_abs_deg = 9.0


        self.init_candidate = None
        self.init_count = 0
        self.init_required_count = 3
        self.init_candidate_dist_m = 0.10
        self.init_candidate_yaw_deg = 12.0

        self.relocalize_candidate = None
        self.relocalize_count = 0
        self.relocalize_required_count_3plus = 5
        self.relocalize_required_count_2 = 8
        self.relocalize_big_jump_m = 0.60
        self.relocalize_candidate_dist_m = 0.12
        self.relocalize_candidate_yaw_deg = 12.0
        self.relocalize_min_marker_count = 2

        self.marker_consistency_filter_enabled = True
        self.marker_consistency_min_count = 3
        self.marker_consistency_dist_m = 0.45
        self.marker_consistency_yaw_deg = 45.0

        self.face_group_filter_enabled = True
        self.face_group_spatial_cluster_m = 0.70
        self.face_group_min_count_for_pnp = 2
        self.face_group_error_reject_px = 10.0
        self.face_group_error_weight = 1.0
        self.face_group_count_bonus = 0.35
        self.face_group_distance_weight = 0.35

        # Multi-hypothesis marker selection.
        # This is the important part for corners: do not blindly keep one wall,
        # and do not blindly use every visible marker either. Build several
        # marker-set hypotheses, solvePnP each one, and choose the pose that is
        # most consistent with the current robot motion.
        self.pnp_hypothesis_enabled = True
        self.pnp_hypothesis_adjacent_cluster_m = 0.60
        self.pnp_hypothesis_max_candidates = 24
        self.pnp_hypothesis_use_all_bonus = 0.10
        self.pnp_hypothesis_combo_bonus = 0.00
        self.pnp_hypothesis_pos_weight_stopped = 12.0
        self.pnp_hypothesis_pos_weight_moving = 8.0
        self.pnp_hypothesis_pos_weight_pivot = 14.0
        self.pnp_hypothesis_yaw_weight_stopped = 0.16
        self.pnp_hypothesis_yaw_weight_moving = 0.12
        self.pnp_hypothesis_yaw_weight_pivot = 0.16
        self.pnp_hypothesis_large_jump_penalty = 999.0
        self.pnp_hypothesis_warn_log = True

        # Face-group lock prevents the pose from jumping to a completely
        # different wall/marker face when the camera sees multiple walls at
        # once. The active face is learned from the first accepted ArUco pose.
        # A different face can be used only after several stable frames and
        # only if it is not a huge map jump.
        self.face_lock_enabled = True
        self.active_face_key = None
        self.active_face_ids = []
        self.pending_face_key = None
        self.pending_face_label = ''
        self.pending_face_ids = []
        self.face_switch_candidate = None
        self.face_switch_count = 0
        self.face_switch_required_count = 3
        self.face_switch_candidate_dist_m = 0.35
        self.face_switch_candidate_yaw_deg = 18.0
        self.face_switch_min_markers = 2
        self.face_switch_max_jump_m = 1.30
        self.face_switch_hard_reject_jump_m = 2.20
        self.face_switch_max_yaw_deg = 70.0
        self.face_switch_max_reproj_px = 3.50

        # SMART V10: face lock is useful, but it must not hold a weak old face
        # when the camera clearly sees a stronger new face.  This prevents
        # cases like active face with only 2 markers while another wall has
        # 5~9 good markers.
        self.smart_face_switch_enabled = True
        self.smart_face_active_weak_count = 2
        self.smart_face_new_strong_count = 5
        self.smart_face_count_advantage = 3
        self.smart_face_immediate_count = 7
        self.smart_face_max_jump_m = 0.65
        self.smart_face_max_yaw_deg = 55.0
        self.smart_face_max_reproj_px = 3.20
        self.smart_face_score_margin = 2.50
        self.smart_face_allow_mixed_all_visible = True

        self.get_logger().info(
            f'face-group lock: enabled={self.face_lock_enabled}, '
            f'switch_required={self.face_switch_required_count}, '
            f'switch_jump<={self.face_switch_max_jump_m:.2f}m, '
            f'hard_reject_jump>={self.face_switch_hard_reject_jump_m:.2f}m'
        )

        self.get_logger().info(
            f'smart face switch: enabled={self.smart_face_switch_enabled}, '
            f'weak_active<={self.smart_face_active_weak_count}, '
            f'strong_new>={self.smart_face_new_strong_count}, '
            f'immediate_new>={self.smart_face_immediate_count}, '
            f'jump<={self.smart_face_max_jump_m:.2f}m, '
            f'yaw<={self.smart_face_max_yaw_deg:.1f}deg, '
            f'reproj<={self.smart_face_max_reproj_px:.1f}px'
        )

        self.reprojection_error_warn_px = 8.0
        self.reprojection_error_reject_px = 25.0

        self.single_rb_enabled = True
        self.single_rb_reprojection_error_px = 10.0
        self.single_rb_max_range_m = 1.80
        self.single_rb_good_error_m = 0.15
        self.single_rb_mid_error_m = 0.30
        self.single_rb_reject_error_m = 0.60
        self.single_rb_good_alpha = 0.10
        self.single_rb_mid_alpha = 0.035
        self.single_rb_yaw_alpha = 0.0

        self.get_logger().info('relative_pose_node FACE_LOCK_SMART_V10 started.')
        self.get_logger().info('Mode: smart face-lock + PnP hypothesis + robust median buffer + safer face switch')
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
        self.get_logger().info(
            'boot init 2-marker allowed: '
            f'reproj<={self.boot_init_2marker_max_reproj_px}, '
            f'spread<={self.boot_init_2marker_max_position_spread_m}/'
            f'{self.boot_init_2marker_max_position_spread_abs_m}m, '
            f'yaw<={self.boot_init_2marker_max_yaw_spread_deg}/'
            f'{self.boot_init_2marker_max_yaw_spread_abs_deg}deg'
        )
        self.get_logger().info(
            'control correction: moving gate relaxed + stationary force relocalize enabled'
        )
        self.get_logger().info(f'relocalize_big_jump_m: {self.relocalize_big_jump_m}')
        self.get_logger().info(f'relocalize_min_marker_count: {self.relocalize_min_marker_count}')
        self.get_logger().info(
            f'robust control pose: buffer={self.raw_pose_buffer.maxlen}, '
            f'min_samples={self.robust_pose_min_samples}, '
            f'max_step={self.max_normal_step_m:.3f}m, '
            f'max_yaw_step={self.max_normal_yaw_step_deg:.1f}deg, '
            f'jump_required_3plus={self.control_jump_required_3plus}, '
            f'jump_required_2={self.control_jump_required_2}'
        )
        self.get_logger().info(
            f'moving high-confidence ArUco correction: '
            f'markers>={self.moving_high_conf_min_markers}, '
            f'reproj<={self.moving_high_conf_max_reproj_px:.1f}px, '
            f'raw_jump<={self.moving_high_conf_max_raw_jump_m:.2f}m, '
            f'raw_yaw<={self.moving_high_conf_max_raw_yaw_deg:.1f}deg'
        )
        self.get_logger().info(
            f'stable 2-marker recovery: stopped_required={self.stable_two_marker_required_stopped}, '
            f'moving_required={self.stable_two_marker_required_moving}, '
            f'max_jump={self.stable_two_marker_max_jump_m:.2f}m, '
            f'max_yaw={self.stable_two_marker_max_yaw_deg:.1f}deg'
        )
        self.get_logger().info(
            f'close-position yaw recovery: markers>={self.close_position_yaw_recovery_min_markers}, '
            f'dist<={self.close_position_yaw_recovery_max_dist_m:.2f}m, '
            f'yaw<={self.close_position_yaw_recovery_max_yaw_deg:.1f}deg'
        )
        self.get_logger().info(
            f'stationary force relocalize: markers>={self.stationary_force_min_markers}, '
            f'required={self.stationary_force_required_count}, '
            f'jump<={self.stationary_force_max_jump_m:.2f}m, '
            f'yaw<={self.stationary_force_max_yaw_deg:.1f}deg, '
            f'reproj<={self.stationary_force_max_reproj_px:.1f}px'
        )
        self.get_logger().info(
            f'single-marker fast recovery: stopped_required={self.single_marker_recovery_required_stopped}, '
            f'moving_required={self.single_marker_recovery_required_moving}, '
            f'max_jump={self.single_marker_recovery_max_jump_m:.2f}m, '
            f'max_yaw={self.single_marker_recovery_max_yaw_deg:.1f}deg'
        )

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

        face_group_blocks = self.select_best_marker_face_group(marker_blocks)
        filtered_blocks = self.filter_marker_blocks_by_consistency(face_group_blocks)

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

        self.raw_x = robot_x
        self.raw_y = robot_y
        self.raw_yaw = robot_yaw

        raw_step_dist = math.hypot(
            robot_x - self.x,
            robot_y - self.y
        )

        raw_step_yaw = abs(
            self.angle_diff(
                robot_yaw,
                self.yaw
            )
        )

        raw_is_large_jump = (
            self.pose_initialized_by_aruco
            and (
                raw_step_dist > self.max_normal_step_m
                or raw_step_yaw > math.radians(
                    self.max_normal_yaw_step_deg
                )
            )
        )

        single_pnp_rejected = (
            marker_count == 1
            and not single_rb_valid
            and pose_mode_cmd == 'single_pnp_rejected:1'
        )

        moving_high_confidence_pose = (
            self.current_cmd != 's'
            and marker_count >= self.moving_high_conf_min_markers
            and reproj_error <= self.moving_high_conf_max_reproj_px
            and raw_step_dist <= self.moving_high_conf_max_raw_jump_m
            and raw_step_yaw <= math.radians(self.moving_high_conf_max_raw_yaw_deg)
        )

        close_position_yaw_recovery_raw_pose = (
            marker_count >= self.close_position_yaw_recovery_min_markers
            and reproj_error <= self.close_position_yaw_recovery_max_reproj_px
            and raw_step_dist <= self.close_position_yaw_recovery_max_dist_m
            and raw_step_yaw <= math.radians(self.close_position_yaw_recovery_max_yaw_deg)
        )

        moving_medium_confidence_pose = (
            self.current_cmd != 's'
            and marker_count >= 2
            and reproj_error <= 3.0
            and raw_step_dist <= 0.60
            and raw_step_yaw <= math.radians(50.0)
        )

        block_raw_buffer_update = (
            single_pnp_rejected
            or (
                raw_is_large_jump
                and self.current_cmd != 's'
                and not moving_high_confidence_pose
                and not moving_medium_confidence_pose
                and not close_position_yaw_recovery_raw_pose
            )
        )

        if block_raw_buffer_update:
            recovered_by_single_marker = (
                marker_count == 1
                and single_rb_valid
                and self.try_single_marker_stable_correction(
                    robot_x,
                    robot_y,
                    robot_yaw,
                    marker_count,
                    main_marker_id,
                    reproj_error=single_rb_result['reproj_error'] if single_rb_result is not None else reproj_error,
                    range_m=single_rb_range if single_rb_result is not None else 9999.0,
                    source_label='blocked_raw_jump'
                )
            )

            if recovered_by_single_marker:
                self.last_robust_x = self.x
                self.last_robust_y = self.y
                self.last_robust_yaw = self.yaw
            else:
                self.aruco_pose_accepted = False

                if self.rejected_pose_clear_buffer:
                    self.raw_pose_buffer.clear()

                if single_pnp_rejected:
                    self.control_jump_candidate = None
                    self.control_jump_candidate_count = 0

                    self.get_logger().warn(
                        f'Rejected pose not added to robust buffer: '
                        f'cmd={pose_mode_cmd}, marker={main_marker_id}'
                    )
                else:
                    self.control_jump_candidate = None
                    self.control_jump_candidate_count = 0

                    self.get_logger().warn(
                        f'Large ArUco jump ignored while robot/search is moving: '
                        f'cmd={self.current_cmd}, '
                        f'raw=({robot_x:.3f},{robot_y:.3f},'
                        f'{math.degrees(robot_yaw):.1f}), '
                        f'current=({self.x:.3f},{self.y:.3f},'
                        f'{math.degrees(self.yaw):.1f}), '
                        f'dist={raw_step_dist:.3f}, '
                        f'yaw={math.degrees(raw_step_yaw):.1f}, '
                        f'markers={marker_count}, marker_id={main_marker_id}'
                    )

        else:
            self.add_raw_pose_sample(
                robot_x,
                robot_y,
                robot_yaw,
                marker_count,
                reproj_error
            )

            robust_pose = self.get_robust_raw_pose()

            if robust_pose is None:
                self.aruco_pose_accepted = False
            else:
                accepted_pose = self.apply_robust_control_pose(
                    robust_pose['x'],
                    robust_pose['y'],
                    robust_pose['yaw'],
                    marker_count,
                    main_marker_id,
                    single_rb_valid=single_rb_valid,
                    reproj_error=robust_pose['reproj_error'],
                    position_spread=robust_pose['position_spread'],
                    yaw_spread_deg=robust_pose['yaw_spread_deg'],
                    max_position_spread=robust_pose['max_position_spread'],
                    max_yaw_spread_deg=robust_pose['max_yaw_spread_deg']
                )

                self.aruco_pose_accepted = accepted_pose

                if accepted_pose:
                    self.apply_pending_face_lock(marker_count)
                    self.last_robust_x = self.x
                    self.last_robust_y = self.y
                    self.last_robust_yaw = self.yaw

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
            reproj_error=reproj_error,
            marker_count_used=marker_count,
            used_ids_text='|'.join(str(mid) for mid in used_marker_ids),
            pnp_label=self.last_selected_hyp_label,
            active_face=str(self.active_face_key)
        )

    def handle_marker_lost(self):
        self.aruco_pose_accepted = False
        self.marker_lost_count += 1

        if self.marker_lost_count >= self.marker_lost_limit:
            self.marker_seen = False
            self.last_marker_id = -1
            self.init_candidate = None
            self.init_count = 0
            self.relocalize_candidate = None
            self.relocalize_count = 0
            self.raw_pose_buffer.clear()
            self.control_jump_candidate = None
            self.control_jump_candidate_count = 0
            self.stationary_recovery_candidate = None
            self.stationary_recovery_count = 0

    def marker_face_key(self, yaw):
        step = math.pi / 2.0
        idx = int(round(self.normalize_angle(yaw) / step))
        idx = ((idx + 2) % 4) - 2
        return idx

    def get_blocks_face_key(self, blocks):
        if blocks is None or len(blocks) <= 0:
            return None, []

        counts = {}
        ids_by_key = {}
        for block in blocks:
            try:
                yaw = float(block['pose'].get('yaw', 0.0))
                key = self.marker_face_key(yaw)
                counts[key] = counts.get(key, 0) + 1
                ids_by_key.setdefault(key, []).append(int(block['id']))
            except Exception:
                continue

        if len(counts) <= 0:
            return None, []

        best_key = sorted(counts.keys(), key=lambda k: (-counts[k], k))[0]
        return best_key, sorted(ids_by_key.get(best_key, []))

    def reset_face_switch_candidate(self):
        self.face_switch_candidate = None
        self.face_switch_count = 0

    def update_face_switch_candidate(self, candidate):
        key = candidate.get('face_key', None)
        if key is None:
            self.reset_face_switch_candidate()
            return 0

        if self.face_switch_candidate is None:
            self.face_switch_candidate = {
                'face_key': key,
                'x': candidate['x'],
                'y': candidate['y'],
                'yaw': candidate['yaw']
            }
            self.face_switch_count = 1
            return self.face_switch_count

        same_key = key == self.face_switch_candidate.get('face_key')
        candidate_dist = math.hypot(
            candidate['x'] - self.face_switch_candidate['x'],
            candidate['y'] - self.face_switch_candidate['y']
        )
        candidate_yaw = abs(
            self.angle_diff(candidate['yaw'], self.face_switch_candidate['yaw'])
        )

        if (
            same_key
            and candidate_dist <= self.face_switch_candidate_dist_m
            and candidate_yaw <= math.radians(self.face_switch_candidate_yaw_deg)
        ):
            self.face_switch_count += 1
            self.face_switch_candidate['x'] = candidate['x']
            self.face_switch_candidate['y'] = candidate['y']
            self.face_switch_candidate['yaw'] = candidate['yaw']
        else:
            self.face_switch_candidate = {
                'face_key': key,
                'x': candidate['x'],
                'y': candidate['y'],
                'yaw': candidate['yaw']
            }
            self.face_switch_count = 1

        return self.face_switch_count

    def apply_pending_face_lock(self, marker_count):
        if not self.face_lock_enabled:
            return

        if marker_count < 2:
            return

        if self.pending_face_key is None:
            return

        if self.active_face_key is None:
            self.active_face_key = self.pending_face_key
            self.active_face_ids = list(self.pending_face_ids)
            self.reset_face_switch_candidate()
            self.get_logger().warn(
                f'FACE LOCK SET: face={self.active_face_key}, '
                f'label={self.pending_face_label}, ids={self.active_face_ids}'
            )
            return

        if self.pending_face_key == self.active_face_key:
            self.active_face_ids = sorted(list(set(self.active_face_ids + list(self.pending_face_ids))))
            self.reset_face_switch_candidate()
            return

        old_face = self.active_face_key
        self.active_face_key = self.pending_face_key
        self.active_face_ids = list(self.pending_face_ids)
        self.reset_face_switch_candidate()
        self.get_logger().warn(
            f'FACE LOCK SWITCHED: {old_face} -> {self.active_face_key}, '
            f'label={self.pending_face_label}, ids={self.active_face_ids}'
        )

    def split_blocks_by_spatial_cluster(self, blocks):
        if len(blocks) <= 1:
            return [blocks]

        centers = []
        for block in blocks:
            pose = block['pose']
            centers.append((float(pose['x']), float(pose['y'])))

        visited = set()
        clusters = []

        for i in range(len(blocks)):
            if i in visited:
                continue

            q = deque([i])
            visited.add(i)
            cluster_idx = []

            while q:
                cur = q.popleft()
                cluster_idx.append(cur)
                cx, cy = centers[cur]

                for j in range(len(blocks)):
                    if j in visited:
                        continue
                    nx, ny = centers[j]
                    if math.hypot(nx - cx, ny - cy) <= self.face_group_spatial_cluster_m:
                        visited.add(j)
                        q.append(j)

            clusters.append([blocks[k] for k in cluster_idx])

        return clusters

    def make_pnp_arrays_from_blocks(self, blocks):
        object_points = []
        image_points = []

        for block in blocks:
            world_corners = block['world_corners']
            img_corners = block['img_corners']

            for j in range(4):
                object_points.append(world_corners[j])
                image_points.append(img_corners[j])

        return (
            np.array(object_points, dtype=np.float64),
            np.array(image_points, dtype=np.float64)
        )

    def block_center_xy(self, block):
        pose = block['pose']
        return float(pose['x']), float(pose['y'])

    def min_distance_between_block_sets(self, blocks_a, blocks_b):
        best = 9999.0
        for a in blocks_a:
            ax, ay = self.block_center_xy(a)
            for b in blocks_b:
                bx, by = self.block_center_xy(b)
                d = math.hypot(ax - bx, ay - by)
                if d < best:
                    best = d
        return best

    def unique_blocks_by_id(self, blocks):
        seen = set()
        out = []
        for b in blocks:
            mid = b['id']
            if mid in seen:
                continue
            seen.add(mid)
            out.append(b)
        return out

    def marker_set_key(self, blocks):
        return tuple(sorted([b['id'] for b in blocks]))

    def make_marker_group_hypotheses(self, marker_blocks):
        hypotheses = []
        seen = set()

        def add(label, blocks):
            blocks = self.unique_blocks_by_id(blocks)
            if len(blocks) <= 0:
                return
            key = self.marker_set_key(blocks)
            if key in seen:
                return
            seen.add(key)
            hypotheses.append({
                'label': label,
                'blocks': blocks,
                'ids': list(key)
            })

        add('all_visible', marker_blocks)

        yaw_groups = {}
        for block in marker_blocks:
            yaw = float(block['pose'].get('yaw', 0.0))
            key = self.marker_face_key(yaw)
            yaw_groups.setdefault(key, []).append(block)

        spatial_clusters = []
        for yaw_key, yaw_blocks in yaw_groups.items():
            add(f'yaw_{yaw_key}_all', yaw_blocks)
            for idx, cluster_blocks in enumerate(self.split_blocks_by_spatial_cluster(yaw_blocks)):
                spatial_clusters.append({
                    'yaw_key': yaw_key,
                    'blocks': cluster_blocks,
                    'ids': [b['id'] for b in cluster_blocks],
                    'label': f'yaw_{yaw_key}_cluster_{idx}'
                })
                add(f'yaw_{yaw_key}_cluster_{idx}', cluster_blocks)

        for i in range(len(spatial_clusters)):
            for j in range(i + 1, len(spatial_clusters)):
                a = spatial_clusters[i]
                b = spatial_clusters[j]
                d = self.min_distance_between_block_sets(a['blocks'], b['blocks'])
                if d <= self.pnp_hypothesis_adjacent_cluster_m:
                    label = f'adjacent_{a["label"]}+{b["label"]}'
                    add(label, a['blocks'] + b['blocks'])

        hypotheses.sort(key=lambda h: (-len(h['blocks']), h['label']))
        return hypotheses[:self.pnp_hypothesis_max_candidates]

    def pnp_hypothesis_motion_penalty(self, x, y, yaw):
        if not self.pose_initialized_by_aruco:
            return 0.0, 0.0, 0.0, False

        pos_jump = math.hypot(x - self.x, y - self.y)
        yaw_jump_deg = abs(self.angle_diff(yaw, self.yaw)) * 180.0 / math.pi

        if self.current_cmd in ['q', 'e']:
            pos_weight = self.pnp_hypothesis_pos_weight_pivot
            yaw_weight = self.pnp_hypothesis_yaw_weight_pivot
            large_jump = pos_jump > 0.75 or yaw_jump_deg > 80.0
        elif self.current_cmd == 'w':
            pos_weight = self.pnp_hypothesis_pos_weight_moving
            yaw_weight = self.pnp_hypothesis_yaw_weight_moving
            large_jump = pos_jump > 0.85 or yaw_jump_deg > 70.0
        elif self.current_cmd in ['a', 'd']:
            pos_weight = self.pnp_hypothesis_pos_weight_moving
            yaw_weight = self.pnp_hypothesis_yaw_weight_moving
            large_jump = pos_jump > 0.85 or yaw_jump_deg > 70.0
        else:
            pos_weight = self.pnp_hypothesis_pos_weight_stopped
            yaw_weight = self.pnp_hypothesis_yaw_weight_stopped
            large_jump = pos_jump > 0.60 or yaw_jump_deg > 70.0

        penalty = pos_jump * pos_weight + yaw_jump_deg * yaw_weight
        if large_jump:
            penalty += self.pnp_hypothesis_large_jump_penalty

        return penalty, pos_jump, yaw_jump_deg, large_jump

    def face_switch_quality_ok(self, candidate, max_jump=None, max_yaw_deg=None, max_reproj=None, min_markers=None):
        if candidate is None:
            return False

        switch_face = candidate.get('face_key', None)
        switch_count = int(candidate.get('count', 0))
        switch_jump = float(candidate.get('pos_jump', 9999.0))
        switch_yaw = float(candidate.get('yaw_jump_deg', 9999.0))
        switch_err = float(candidate.get('err', 9999.0))

        if max_jump is None:
            max_jump = self.face_switch_max_jump_m
        if max_yaw_deg is None:
            max_yaw_deg = self.face_switch_max_yaw_deg
        if max_reproj is None:
            max_reproj = self.face_switch_max_reproj_px
        if min_markers is None:
            min_markers = self.face_switch_min_markers

        return (
            switch_face is not None
            and switch_count >= min_markers
            and switch_jump <= max_jump
            and switch_jump < self.face_switch_hard_reject_jump_m
            and switch_yaw <= max_yaw_deg
            and switch_err <= max_reproj
        )

    def is_smart_face_switch_candidate(self, same_face_candidate, other_candidate):
        if not self.smart_face_switch_enabled:
            return False, 'disabled'

        if other_candidate is None:
            return False, 'no_other'

        if not self.face_switch_quality_ok(
            other_candidate,
            max_jump=self.smart_face_max_jump_m,
            max_yaw_deg=self.smart_face_max_yaw_deg,
            max_reproj=self.smart_face_max_reproj_px,
            min_markers=self.face_switch_min_markers
        ):
            return False, 'quality'

        other_count = int(other_candidate.get('count', 0))
        other_label = str(other_candidate.get('label', ''))

        if same_face_candidate is None:
            if other_count >= self.smart_face_immediate_count:
                return True, 'no_active_candidate_and_many_markers'
            if (
                other_count >= self.smart_face_new_strong_count
                and float(other_candidate.get('pos_jump', 9999.0)) <= self.smart_face_max_jump_m
            ):
                return True, 'no_active_candidate_and_strong'
            return False, 'no_active_candidate_wait'

        same_count = int(same_face_candidate.get('count', 0))
        same_score = float(same_face_candidate.get('score', 9999.0))
        other_score = float(other_candidate.get('score', 9999.0))
        count_advantage = other_count - same_count

        active_weak_new_strong = (
            same_count <= self.smart_face_active_weak_count
            and other_count >= self.smart_face_new_strong_count
        )

        large_count_advantage = count_advantage >= self.smart_face_count_advantage
        much_better_score = other_score <= same_score + self.smart_face_score_margin
        immediate_many_markers = other_count >= self.smart_face_immediate_count

        if immediate_many_markers and much_better_score:
            return True, 'immediate_many_markers'

        if active_weak_new_strong and (large_count_advantage or much_better_score):
            return True, 'weak_active_strong_new'

        if large_count_advantage and much_better_score and other_count >= self.smart_face_new_strong_count:
            return True, 'count_advantage'

        if (
            self.smart_face_allow_mixed_all_visible
            and other_label == 'all_visible'
            and other_count >= self.smart_face_new_strong_count
            and same_count <= self.smart_face_active_weak_count
            and much_better_score
        ):
            return True, 'all_visible_stronger'

        return False, 'not_stronger'

    def select_best_marker_face_group(self, marker_blocks):
        if not self.face_group_filter_enabled or not self.pnp_hypothesis_enabled:
            return marker_blocks

        if len(marker_blocks) <= 1:
            return marker_blocks

        hypotheses = self.make_marker_group_hypotheses(marker_blocks)
        if len(hypotheses) <= 1:
            return marker_blocks

        candidates = []

        for hyp in hypotheses:
            blocks = hyp['blocks']
            marker_count = len(blocks)
            ids = [b['id'] for b in blocks]

            if marker_count < self.face_group_min_count_for_pnp:
                continue

            if not self.pose_initialized_by_aruco:
                if marker_count < self.boot_init_min_markers:
                    continue
                if hyp['label'] == 'all_visible' or hyp['label'].startswith('adjacent_'):
                    continue

            object_points, image_points = self.make_pnp_arrays_from_blocks(blocks)
            success, rvec, tvec, err = self.solve_pnp_best(
                object_points,
                image_points,
                marker_count
            )

            if not success:
                continue

            if err > self.face_group_error_reject_px:
                if self.pnp_hypothesis_warn_log:
                    self.get_logger().warn(
                        f'PnP hypothesis rejected by reprojection: '
                        f'label={hyp["label"]}, ids={ids}, err={err:.2f}px'
                    )
                continue

            result = self.compute_robot_pose_from_map_pnp(rvec, tvec)
            robot_x = result['robot_x'] + self.correction['x_offset']
            robot_y = result['robot_y'] + self.correction['y_offset']
            robot_yaw = self.normalize_angle(
                result['robot_yaw'] + self.correction['yaw_offset']
            )

            motion_penalty, pos_jump, yaw_jump_deg, large_jump = self.pnp_hypothesis_motion_penalty(
                robot_x,
                robot_y,
                robot_yaw
            )

            if self.pose_initialized_by_aruco and large_jump:
                if self.pnp_hypothesis_warn_log:
                    self.get_logger().warn(
                        f'PnP hypothesis rejected by motion gate: '
                        f'label={hyp["label"]}, ids={ids}, '
                        f'jump={pos_jump:.3f}m/{yaw_jump_deg:.1f}deg, err={err:.2f}px'
                    )
                continue

            score = 0.0
            score += self.face_group_error_weight * err
            score += motion_penalty
            score -= self.face_group_count_bonus * min(marker_count, 8)

            if hyp['label'] == 'all_visible':
                score -= self.pnp_hypothesis_use_all_bonus
            elif hyp['label'].startswith('adjacent_'):
                score -= self.pnp_hypothesis_combo_bonus

            face_key, face_ids = self.get_blocks_face_key(blocks)

            candidates.append({
                'label': hyp['label'],
                'blocks': blocks,
                'ids': ids,
                'count': marker_count,
                'err': err,
                'score': score,
                'x': robot_x,
                'y': robot_y,
                'yaw': robot_yaw,
                'pos_jump': pos_jump,
                'yaw_jump_deg': yaw_jump_deg,
                'large_jump': large_jump,
                'face_key': face_key,
                'face_ids': face_ids
            })

        if len(candidates) == 0:
            if self.pose_initialized_by_aruco:
                return []
            return marker_blocks

        candidates.sort(key=lambda c: c['score'])

        if (
            self.face_lock_enabled
            and self.pose_initialized_by_aruco
            and self.active_face_key is not None
        ):
            same_face_candidates = [
                c for c in candidates
                if c.get('face_key', None) == self.active_face_key
            ]
            other_face_candidates = [
                c for c in candidates
                if c.get('face_key', None) != self.active_face_key
            ]

            same_face_candidates.sort(key=lambda c: c['score'])
            other_face_candidates.sort(key=lambda c: c['score'])

            best_same = same_face_candidates[0] if len(same_face_candidates) > 0 else None
            best_other = other_face_candidates[0] if len(other_face_candidates) > 0 else None

            smart_switch, smart_reason = self.is_smart_face_switch_candidate(best_same, best_other)

            if smart_switch:
                old_active = self.active_face_key
                self.active_face_key = best_other.get('face_key', None)
                self.active_face_ids = list(best_other.get('face_ids', []))
                self.reset_face_switch_candidate()
                self.get_logger().warn(
                    f'SMART FACE SWITCH IMMEDIATE: {old_active} -> {self.active_face_key}, '
                    f'reason={smart_reason}, label={best_other["label"]}, '
                    f'ids={best_other["ids"]}, count={best_other["count"]}, '
                    f'jump={best_other["pos_jump"]:.3f}m/{best_other["yaw_jump_deg"]:.1f}deg, '
                    f'err={best_other["err"]:.2f}px, score={best_other["score"]:.2f}, '
                    f'old_same_count={best_same["count"] if best_same is not None else 0}, '
                    f'old_same_score={best_same["score"] if best_same is not None else 9999.0:.2f}'
                )
                candidates = [best_other] + [
                    c for c in candidates
                    if c is not best_other and c.get('face_key', None) == self.active_face_key
                ]
                candidates.sort(key=lambda c: c['score'])
            elif len(same_face_candidates) > 0:
                if len(same_face_candidates) != len(candidates):
                    removed_faces = sorted(list(set([
                        str(c.get('face_key', None)) for c in candidates
                        if c.get('face_key', None) != self.active_face_key
                    ])))
                    self.get_logger().warn(
                        f'FACE LOCK FILTER: active={self.active_face_key}, '
                        f'kept={len(same_face_candidates)}, '
                        f'removed_faces={removed_faces}, '
                        f'best_same_count={best_same["count"]}, '
                        f'best_other_count={best_other["count"] if best_other is not None else 0}, '
                        f'smart_reason={smart_reason}'
                    )
                candidates = same_face_candidates
                candidates.sort(key=lambda c: c['score'])
                self.reset_face_switch_candidate()
            else:
                switch_candidate = candidates[0]
                switch_jump = switch_candidate.get('pos_jump', 9999.0)
                switch_yaw = switch_candidate.get('yaw_jump_deg', 9999.0)
                switch_err = switch_candidate.get('err', 9999.0)
                switch_count = switch_candidate.get('count', 0)
                switch_face = switch_candidate.get('face_key', None)

                switch_allowed_quality = self.face_switch_quality_ok(switch_candidate)

                if not switch_allowed_quality:
                    self.get_logger().warn(
                        f'FACE LOCK REJECT: active={self.active_face_key}, '
                        f'candidate_face={switch_face}, label={switch_candidate["label"]}, '
                        f'ids={switch_candidate["ids"]}, '
                        f'jump={switch_jump:.3f}m/{switch_yaw:.1f}deg, '
                        f'err={switch_err:.2f}px, count={switch_count}'
                    )
                    return []

                smart_no_same, smart_reason_no_same = self.is_smart_face_switch_candidate(None, switch_candidate)
                if smart_no_same:
                    old_active = self.active_face_key
                    self.active_face_key = switch_face
                    self.active_face_ids = list(switch_candidate.get('face_ids', []))
                    self.reset_face_switch_candidate()
                    self.get_logger().warn(
                        f'SMART FACE SWITCH NO-SAME: {old_active} -> {self.active_face_key}, '
                        f'reason={smart_reason_no_same}, label={switch_candidate["label"]}, '
                        f'ids={switch_candidate["ids"]}, count={switch_count}, '
                        f'jump={switch_jump:.3f}m/{switch_yaw:.1f}deg, err={switch_err:.2f}px'
                    )
                else:
                    switch_seen = self.update_face_switch_candidate(switch_candidate)
                    if switch_seen < self.face_switch_required_count:
                        self.get_logger().warn(
                            f'FACE SWITCH WAIT: active={self.active_face_key}, '
                            f'candidate_face={switch_face}, '
                            f'{switch_seen}/{self.face_switch_required_count}, '
                            f'label={switch_candidate["label"]}, ids={switch_candidate["ids"]}, '
                            f'jump={switch_jump:.3f}m/{switch_yaw:.1f}deg, '
                            f'err={switch_err:.2f}px, smart_reason={smart_reason_no_same}'
                        )
                        return []

                    old_active = self.active_face_key
                    self.active_face_key = switch_face
                    self.active_face_ids = list(switch_candidate.get('face_ids', []))
                    self.reset_face_switch_candidate()
                    self.get_logger().warn(
                        f'FACE SWITCH GATE OK: {old_active} -> {self.active_face_key}, '
                        f'label={switch_candidate["label"]}, ids={switch_candidate["ids"]}, '
                        f'jump={switch_jump:.3f}m/{switch_yaw:.1f}deg, '
                        f'err={switch_err:.2f}px'
                    )

        best = candidates[0]
        original_ids = [b['id'] for b in marker_blocks]
        removed_ids = [mid for mid in original_ids if mid not in best['ids']]

        if len(candidates) >= 2:
            second = candidates[1]
            if (
                best['ids'] != original_ids
                or best['label'] != 'all_visible'
                or best['large_jump']
                or removed_ids
            ):
                self.get_logger().warn(
                    f'PnP hypothesis selected: label={best["label"]}, '
                    f'ids={best["ids"]}, err={best["err"]:.2f}px, '
                    f'count={best["count"]}, score={best["score"]:.2f}, '
                    f'jump={best["pos_jump"]:.3f}m/{best["yaw_jump_deg"]:.1f}deg, '
                    f'removed={removed_ids}, '
                    f'second={second["label"]}:{second["score"]:.2f}'
                )
        elif removed_ids:
            self.get_logger().warn(
                f'PnP hypothesis selected: label={best["label"]}, '
                f'ids={best["ids"]}, err={best["err"]:.2f}px, '
                f'count={best["count"]}, score={best["score"]:.2f}, '
                f'jump={best["pos_jump"]:.3f}m/{best["yaw_jump_deg"]:.1f}deg, '
                f'removed={removed_ids}'
            )

        self.pending_face_key = best.get('face_key', None)
        self.pending_face_label = best.get('label', '')
        self.pending_face_ids = list(best.get('face_ids', []))
        self.last_selected_hyp_label = str(best.get('label', ''))
        self.last_selected_ids_text = '|'.join(str(mid) for mid in best.get('ids', []))
        self.last_candidate_count = len(candidates)
        self.last_active_face_key_text = str(self.active_face_key)

        return best['blocks']

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

    def add_raw_pose_sample(
        self,
        raw_x,
        raw_y,
        raw_yaw,
        marker_count,
        reproj_error
    ):
        self.raw_pose_buffer.append({
            'x': float(raw_x),
            'y': float(raw_y),
            'yaw': self.normalize_angle(float(raw_yaw)),
            'marker_count': int(marker_count),
            'reproj_error': float(reproj_error)
        })

    def get_robust_raw_pose(self):
        if len(self.raw_pose_buffer) < self.robust_pose_min_samples:
            return None

        samples = list(self.raw_pose_buffer)

        xs = [sample['x'] for sample in samples]
        ys = [sample['y'] for sample in samples]

        median_x = float(np.median(xs))
        median_y = float(np.median(ys))

        # First estimate a circular center, then unwrap each yaw around it.
        circular_center = self.circular_mean(
            [sample['yaw'] for sample in samples]
        )

        unwrapped_yaws = [
            circular_center
            + self.angle_diff(sample['yaw'], circular_center)
            for sample in samples
        ]

        median_yaw = self.normalize_angle(
            float(np.median(unwrapped_yaws))
        )

        distances = [
            math.hypot(
                sample['x'] - median_x,
                sample['y'] - median_y
            )
            for sample in samples
        ]

        yaw_diffs = [
            abs(self.angle_diff(sample['yaw'], median_yaw))
            for sample in samples
        ]

        position_spread = float(np.median(distances))
        max_position_spread = float(max(distances)) if len(distances) > 0 else 0.0

        yaw_spread_deg = math.degrees(
            float(np.median(yaw_diffs))
        )
        max_yaw_spread_deg = math.degrees(
            float(max(yaw_diffs)) if len(yaw_diffs) > 0 else 0.0
        )

        marker_count = int(round(float(np.median(
            [sample['marker_count'] for sample in samples]
        ))))

        reproj_error = float(np.median(
            [sample['reproj_error'] for sample in samples]
        ))

        return {
            'x': median_x,
            'y': median_y,
            'yaw': median_yaw,
            'position_spread': position_spread,
            'max_position_spread': max_position_spread,
            'yaw_spread_deg': yaw_spread_deg,
            'max_yaw_spread_deg': max_yaw_spread_deg,
            'marker_count': marker_count,
            'reproj_error': reproj_error
        }

    def apply_robust_control_pose(
        self,
        robust_x,
        robust_y,
        robust_yaw,
        marker_count,
        main_marker_id,
        single_rb_valid=False,
        reproj_error=9999.0,
        position_spread=9999.0,
        yaw_spread_deg=9999.0,
        max_position_spread=9999.0,
        max_yaw_spread_deg=9999.0
    ):
        if not self.pose_initialized_by_aruco:
            if marker_count >= 3:
                boot_ok = (
                    reproj_error <= self.boot_init_max_reproj_px
                    and position_spread <= self.boot_init_max_position_spread_m
                    and max_position_spread <= self.boot_init_max_position_spread_abs_m
                    and yaw_spread_deg <= self.boot_init_max_yaw_spread_deg
                    and max_yaw_spread_deg <= self.boot_init_max_yaw_spread_abs_deg
                )
                boot_mode = '3plus'
            elif marker_count == 2:
                boot_ok = (
                    reproj_error <= self.boot_init_2marker_max_reproj_px
                    and position_spread <= self.boot_init_2marker_max_position_spread_m
                    and max_position_spread <= self.boot_init_2marker_max_position_spread_abs_m
                    and yaw_spread_deg <= self.boot_init_2marker_max_yaw_spread_deg
                    and max_yaw_spread_deg <= self.boot_init_2marker_max_yaw_spread_abs_deg
                )
                boot_mode = '2marker'
            else:
                boot_ok = False
                boot_mode = 'need_2plus'

            if not boot_ok:
                self.aruco_pose_accepted = False
                self.get_logger().warn(
                    f'BOOT_INIT WAIT[{boot_mode}]: markers={marker_count}, '
                    f'marker_id={main_marker_id}, '
                    f'reproj={reproj_error:.2f}, '
                    f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                    f'yaw_spread={yaw_spread_deg:.1f}/{max_yaw_spread_deg:.1f}'
                )
                return False

            self.get_logger().warn(
                f'BOOT_INIT GATE OK[{boot_mode}]: markers={marker_count}, '
                f'marker_id={main_marker_id}, '
                f'reproj={reproj_error:.2f}, '
                f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                f'yaw_spread={yaw_spread_deg:.1f}/{max_yaw_spread_deg:.1f}'
            )

            return self.apply_aruco_pose_with_confidence(
                robust_x,
                robust_y,
                robust_yaw,
                marker_count,
                main_marker_id,
                single_rb_valid=single_rb_valid
            )

        step_dist = math.hypot(
            robust_x - self.x,
            robust_y - self.y
        )

        step_yaw = abs(
            self.angle_diff(
                robust_yaw,
                self.yaw
            )
        )

        normal_move = (
            step_dist <= self.max_normal_step_m
            and step_yaw <= math.radians(
                self.max_normal_yaw_step_deg
            )
        )

        if normal_move:
            self.control_jump_candidate = None
            self.control_jump_candidate_count = 0

            return self.apply_aruco_pose_with_confidence(
                robust_x,
                robust_y,
                robust_yaw,
                marker_count,
                main_marker_id,
                single_rb_valid=single_rb_valid
            )

        return self.handle_control_jump_candidate(
            robust_x,
            robust_y,
            robust_yaw,
            marker_count,
            reproj_error,
            position_spread,
            yaw_spread_deg,
            max_position_spread,
            max_yaw_spread_deg
        )

    def try_single_marker_stable_correction(
        self,
        new_x,
        new_y,
        new_yaw,
        marker_count,
        main_marker_id,
        reproj_error=9999.0,
        range_m=9999.0,
        source_label='single_marker'
    ):
        if marker_count != 1:
            self.single_marker_recovery_candidate = None
            self.single_marker_recovery_count = 0
            return False

        jump_dist = math.hypot(new_x - self.x, new_y - self.y)
        jump_yaw = abs(self.angle_diff(new_yaw, self.yaw))

        if not (
            jump_dist <= self.single_marker_recovery_max_jump_m
            and jump_yaw <= math.radians(self.single_marker_recovery_max_yaw_deg)
            and reproj_error <= self.single_marker_recovery_max_reproj_px
            and range_m <= self.single_marker_recovery_max_range_m
        ):
            self.single_marker_recovery_candidate = None
            self.single_marker_recovery_count = 0
            return False

        if self.single_marker_recovery_candidate is None:
            self.single_marker_recovery_candidate = {
                'x': new_x,
                'y': new_y,
                'yaw': new_yaw,
                'marker_id': main_marker_id
            }
            self.single_marker_recovery_count = 1
        else:
            candidate_dist = math.hypot(
                new_x - self.single_marker_recovery_candidate['x'],
                new_y - self.single_marker_recovery_candidate['y']
            )
            candidate_yaw = abs(
                self.angle_diff(new_yaw, self.single_marker_recovery_candidate['yaw'])
            )
            same_marker = (
                main_marker_id == self.single_marker_recovery_candidate.get('marker_id', main_marker_id)
                or main_marker_id < 0
                or self.single_marker_recovery_candidate.get('marker_id', main_marker_id) < 0
            )

            if (
                same_marker
                and candidate_dist <= self.single_marker_recovery_candidate_dist_m
                and candidate_yaw <= math.radians(self.single_marker_recovery_candidate_yaw_deg)
            ):
                self.single_marker_recovery_count += 1
                self.single_marker_recovery_candidate['x'] = new_x
                self.single_marker_recovery_candidate['y'] = new_y
                self.single_marker_recovery_candidate['yaw'] = new_yaw
                self.single_marker_recovery_candidate['marker_id'] = main_marker_id
            else:
                self.single_marker_recovery_candidate = {
                    'x': new_x,
                    'y': new_y,
                    'yaw': new_yaw,
                    'marker_id': main_marker_id
                }
                self.single_marker_recovery_count = 1

        required_count = (
            self.single_marker_recovery_required_stopped
            if self.current_cmd == 's'
            else self.single_marker_recovery_required_moving
        )

        if self.single_marker_recovery_count < required_count:
            self.aruco_pose_accepted = False
            self.get_logger().warn(
                f'SINGLE-MARKER RECOVERY WAIT: '
                f'{self.single_marker_recovery_count}/{required_count}, '
                f'marker={main_marker_id}, '
                f'dist={jump_dist:.3f}, '
                f'yaw={math.degrees(jump_yaw):.1f}, '
                f'reproj={reproj_error:.2f}, '
                f'range={range_m:.3f}, '
                f'source={source_label}'
            )
            return False

        old_x = self.x
        old_y = self.y
        old_yaw = self.yaw

        if self.current_cmd == 's':
            pos_alpha = self.single_marker_recovery_pos_alpha_stopped
            yaw_alpha = self.single_marker_recovery_yaw_alpha_stopped
            correction_type = 'SINGLE-MARKER STOPPED FAST RECOVERY'
        else:
            pos_alpha = self.single_marker_recovery_pos_alpha_moving
            yaw_alpha = self.single_marker_recovery_yaw_alpha_moving
            correction_type = 'SINGLE-MARKER MOVING PARTIAL CORRECTION'

        self.x = (1.0 - pos_alpha) * self.x + pos_alpha * new_x
        self.y = (1.0 - pos_alpha) * self.y + pos_alpha * new_y
        self.yaw = self.smooth_angle(self.yaw, new_yaw, yaw_alpha)

        self.pose_initialized_by_aruco = True
        self.aruco_pose_accepted = True
        self.control_jump_candidate = None
        self.control_jump_candidate_count = 0
        self.relocalize_candidate = None
        self.relocalize_count = 0
        self.stationary_recovery_candidate = None
        self.stationary_recovery_count = 0
        self.stable_two_marker_candidate = None
        self.stable_two_marker_count = 0
        self.raw_pose_buffer.clear()
        self.add_raw_pose_sample(self.x, self.y, self.yaw, marker_count, reproj_error)

        self.get_logger().warn(
            f'{correction_type} ACCEPTED: '
            f'old=({old_x:.3f},{old_y:.3f},{math.degrees(old_yaw):.1f}), '
            f'candidate=({new_x:.3f},{new_y:.3f},{math.degrees(new_yaw):.1f}), '
            f'new=({self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f}), '
            f'dist={jump_dist:.3f}, '
            f'yaw={math.degrees(jump_yaw):.1f}, '
            f'marker={main_marker_id}, '
            f'reproj={reproj_error:.2f}, '
            f'range={range_m:.3f}, '
            f'count={self.single_marker_recovery_count}, '
            f'alpha=({pos_alpha:.2f},{yaw_alpha:.2f}), '
            f'source={source_label}'
        )
        return True

    def pose_inside_marker_bounds(self, x, y):
        if not self.aruco_marker_pose:
            return True

        xs = [float(v['x']) for v in self.aruco_marker_pose.values()]
        ys = [float(v['y']) for v in self.aruco_marker_pose.values()]
        margin = self.stationary_force_map_margin_m

        return (
            min(xs) - margin <= x <= max(xs) + margin
            and min(ys) - margin <= y <= max(ys) + margin
        )

    def try_stationary_force_relocalize(
        self,
        new_x,
        new_y,
        new_yaw,
        marker_count,
        reproj_error=9999.0,
        position_spread=9999.0,
        yaw_spread_deg=9999.0,
        max_position_spread=9999.0,
        max_yaw_spread_deg=9999.0,
        reason='control_jump'
    ):
        if self.current_cmd != 's':
            self.stationary_force_candidate = None
            self.stationary_force_count = 0
            return False

        if marker_count < self.stationary_force_min_markers:
            self.stationary_force_candidate = None
            self.stationary_force_count = 0
            return False

        jump_dist = math.hypot(new_x - self.x, new_y - self.y)
        jump_yaw = abs(self.angle_diff(new_yaw, self.yaw))

        reproj_limit = (
            self.stationary_force_max_reproj_px_2marker
            if marker_count == 2
            else self.stationary_force_max_reproj_px
        )

        spread_ok = (
            marker_count >= self.stationary_force_prefer_markers
            or (
                max_position_spread <= self.stationary_force_max_spread_m
                and max_yaw_spread_deg <= self.stationary_force_max_yaw_spread_deg
            )
        )

        allowed = (
            jump_dist <= self.stationary_force_max_jump_m
            and jump_yaw <= math.radians(self.stationary_force_max_yaw_deg)
            and reproj_error <= reproj_limit
            and spread_ok
            and self.pose_inside_marker_bounds(new_x, new_y)
        )

        if not allowed:
            self.stationary_force_candidate = None
            self.stationary_force_count = 0
            return False

        if self.stationary_force_candidate is None:
            self.stationary_force_candidate = {
                'x': new_x,
                'y': new_y,
                'yaw': new_yaw,
                'marker_count': marker_count
            }
            self.stationary_force_count = 1
        else:
            candidate_dist = math.hypot(
                new_x - self.stationary_force_candidate['x'],
                new_y - self.stationary_force_candidate['y']
            )
            candidate_yaw = abs(
                self.angle_diff(new_yaw, self.stationary_force_candidate['yaw'])
            )

            same_candidate = (
                candidate_dist <= self.stationary_force_candidate_dist_m
                and candidate_yaw <= math.radians(self.stationary_force_candidate_yaw_deg)
            )

            if same_candidate:
                self.stationary_force_count += 1
                self.stationary_force_candidate['x'] = new_x
                self.stationary_force_candidate['y'] = new_y
                self.stationary_force_candidate['yaw'] = new_yaw
                self.stationary_force_candidate['marker_count'] = marker_count
            else:
                self.stationary_force_candidate = {
                    'x': new_x,
                    'y': new_y,
                    'yaw': new_yaw,
                    'marker_count': marker_count
                }
                self.stationary_force_count = 1

        if self.stationary_force_count < self.stationary_force_required_count:
            self.aruco_pose_accepted = False
            self.get_logger().warn(
                f'STATIONARY FORCE RELOCALIZE WAIT: '
                f'{self.stationary_force_count}/{self.stationary_force_required_count}, '
                f'dist={jump_dist:.3f}, '
                f'yaw={math.degrees(jump_yaw):.1f}, '
                f'markers={marker_count}, '
                f'reproj={reproj_error:.2f}, '
                f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                f'reason={reason}'
            )
            return False

        old_x = self.x
        old_y = self.y
        old_yaw = self.yaw

        pos_alpha = self.stationary_force_pos_alpha
        yaw_alpha = self.stationary_force_yaw_alpha

        self.x = (1.0 - pos_alpha) * self.x + pos_alpha * new_x
        self.y = (1.0 - pos_alpha) * self.y + pos_alpha * new_y
        self.yaw = self.smooth_angle(self.yaw, new_yaw, yaw_alpha)

        self.pose_initialized_by_aruco = True
        self.aruco_pose_accepted = True
        self.last_source = 'stationary_force_relocalize'
        self.control_jump_candidate = None
        self.control_jump_candidate_count = 0
        self.relocalize_candidate = None
        self.relocalize_count = 0
        self.stationary_recovery_candidate = None
        self.stationary_recovery_count = 0
        self.stable_two_marker_candidate = None
        self.stable_two_marker_count = 0
        self.single_marker_recovery_candidate = None
        self.single_marker_recovery_count = 0
        self.stationary_force_candidate = None
        self.stationary_force_count = 0
        self.raw_pose_buffer.clear()
        self.add_raw_pose_sample(self.x, self.y, self.yaw, marker_count, reproj_error)

        self.get_logger().warn(
            f'STATIONARY FORCE RELOCALIZE APPLIED: '
            f'old=({old_x:.3f},{old_y:.3f},{math.degrees(old_yaw):.1f}), '
            f'target=({new_x:.3f},{new_y:.3f},{math.degrees(new_yaw):.1f}), '
            f'new=({self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f}), '
            f'dist={jump_dist:.3f}, '
            f'yaw={math.degrees(jump_yaw):.1f}, '
            f'markers={marker_count}, '
            f'reproj={reproj_error:.2f}, '
            f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
            f'count={self.stationary_force_required_count}, '
            f'reason={reason}'
        )
        return True

    def handle_control_jump_candidate(
        self,
        new_x,
        new_y,
        new_yaw,
        marker_count,
        reproj_error=9999.0,
        position_spread=9999.0,
        yaw_spread_deg=9999.0,
        max_position_spread=9999.0,
        max_yaw_spread_deg=9999.0
    ):
        jump_dist = math.hypot(
            new_x - self.x,
            new_y - self.y
        )

        jump_yaw = abs(
            self.angle_diff(
                new_yaw,
                self.yaw
            )
        )

        if self.try_stationary_force_relocalize(
            new_x,
            new_y,
            new_yaw,
            marker_count,
            reproj_error,
            position_spread,
            yaw_spread_deg,
            max_position_spread,
            max_yaw_spread_deg,
            reason='control_jump_before_gate'
        ):
            return True

        if marker_count == 1:
            return self.try_single_marker_stable_correction(
                new_x,
                new_y,
                new_yaw,
                marker_count,
                main_marker_id=-1,
                reproj_error=reproj_error,
                range_m=self.single_marker_recovery_max_range_m,
                source_label='control_jump'
            )

        moving_trusted_correction = (
            self.current_cmd != 's'
            and marker_count >= self.moving_high_conf_min_markers
            and jump_dist <= self.moving_high_conf_max_robust_jump_m
            and jump_yaw <= math.radians(self.moving_high_conf_max_robust_yaw_deg)
            and reproj_error <= self.moving_high_conf_max_reproj_px
            and max_position_spread <= 0.28
            and yaw_spread_deg <= 18.0
        )

        if moving_trusted_correction:
            old_x = self.x
            old_y = self.y
            old_yaw = self.yaw

            pos_alpha = self.moving_high_conf_pos_alpha
            yaw_alpha = self.moving_high_conf_yaw_alpha

            self.x = (1.0 - pos_alpha) * self.x + pos_alpha * new_x
            self.y = (1.0 - pos_alpha) * self.y + pos_alpha * new_y
            self.yaw = self.smooth_angle(self.yaw, new_yaw, yaw_alpha)

            self.control_jump_candidate = None
            self.control_jump_candidate_count = 0
            self.relocalize_candidate = None
            self.relocalize_count = 0
            self.stationary_recovery_candidate = None
            self.stationary_recovery_count = 0

            self.get_logger().warn(
                f'MOVING TRUSTED ARUCO CORRECTION ACCEPTED: '
                f'old=({old_x:.3f},{old_y:.3f},'
                f'{math.degrees(old_yaw):.1f}), '
                f'candidate=({new_x:.3f},{new_y:.3f},'
                f'{math.degrees(new_yaw):.1f}), '
                f'new=({self.x:.3f},{self.y:.3f},'
                f'{math.degrees(self.yaw):.1f}), '
                f'dist={jump_dist:.3f}, '
                f'yaw={math.degrees(jump_yaw):.1f}, '
                f'markers={marker_count}, '
                f'reproj={reproj_error:.2f}, '
                f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                f'alpha=({pos_alpha:.2f},{yaw_alpha:.2f})'
            )

            return True

        close_position_yaw_recovery = (
            marker_count >= self.close_position_yaw_recovery_min_markers
            and jump_dist <= self.close_position_yaw_recovery_max_dist_m
            and jump_yaw <= math.radians(self.close_position_yaw_recovery_max_yaw_deg)
            and reproj_error <= self.close_position_yaw_recovery_max_reproj_px
            and max_position_spread <= self.close_position_yaw_recovery_max_spread_m
            and max_yaw_spread_deg <= self.close_position_yaw_recovery_max_yaw_spread_deg
        )

        if close_position_yaw_recovery:
            old_x = self.x
            old_y = self.y
            old_yaw = self.yaw

            if self.current_cmd == 's':
                pos_alpha = self.close_position_yaw_recovery_pos_alpha_stopped
                yaw_alpha = self.close_position_yaw_recovery_yaw_alpha_stopped
                correction_type = 'CLOSE-POSITION YAW RECOVERY'
            else:
                pos_alpha = self.close_position_yaw_recovery_pos_alpha_moving
                yaw_alpha = self.close_position_yaw_recovery_yaw_alpha_moving
                correction_type = 'MOVING CLOSE-POSITION YAW CORRECTION'

            self.x = (1.0 - pos_alpha) * self.x + pos_alpha * new_x
            self.y = (1.0 - pos_alpha) * self.y + pos_alpha * new_y
            self.yaw = self.smooth_angle(self.yaw, new_yaw, yaw_alpha)

            self.pose_initialized_by_aruco = True
            self.control_jump_candidate = None
            self.control_jump_candidate_count = 0
            self.relocalize_candidate = None
            self.relocalize_count = 0
            self.stationary_recovery_candidate = None
            self.stationary_recovery_count = 0
            self.stable_two_marker_candidate = None
            self.stable_two_marker_count = 0

            self.get_logger().warn(
                f'{correction_type} ACCEPTED: '
                f'old=({old_x:.3f},{old_y:.3f},'
                f'{math.degrees(old_yaw):.1f}), '
                f'candidate=({new_x:.3f},{new_y:.3f},'
                f'{math.degrees(new_yaw):.1f}), '
                f'new=({self.x:.3f},{self.y:.3f},'
                f'{math.degrees(self.yaw):.1f}), '
                f'dist={jump_dist:.3f}, '
                f'yaw={math.degrees(jump_yaw):.1f}, '
                f'markers={marker_count}, '
                f'reproj={reproj_error:.2f}, '
                f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                f'alpha=({pos_alpha:.2f},{yaw_alpha:.2f})'
            )

            return True

        stationary_recovery_allowed = (
            self.current_cmd == 's'
            and marker_count >= self.stationary_recovery_min_markers
            and jump_dist <= self.stationary_recovery_max_jump_m
            and jump_yaw <= math.radians(self.stationary_recovery_max_yaw_deg)
            and reproj_error <= self.stationary_recovery_max_reproj_px
            and position_spread <= self.stationary_recovery_max_spread_m
            and max_position_spread <= self.stationary_recovery_max_spread_m * 2.0
            and yaw_spread_deg <= self.stationary_recovery_max_yaw_spread_deg
            and max_yaw_spread_deg <= self.stationary_recovery_max_yaw_spread_deg * 2.0
        )

        if stationary_recovery_allowed:
            if self.stationary_recovery_candidate is None:
                self.stationary_recovery_candidate = {
                    'x': new_x,
                    'y': new_y,
                    'yaw': new_yaw
                }
                self.stationary_recovery_count = 1
            else:
                candidate_dist = math.hypot(
                    new_x - self.stationary_recovery_candidate['x'],
                    new_y - self.stationary_recovery_candidate['y']
                )

                candidate_yaw = abs(
                    self.angle_diff(
                        new_yaw,
                        self.stationary_recovery_candidate['yaw']
                    )
                )

                if (
                    candidate_dist <= self.stationary_recovery_candidate_dist_m
                    and candidate_yaw <= math.radians(self.stationary_recovery_candidate_yaw_deg)
                ):
                    self.stationary_recovery_count += 1
                else:
                    self.stationary_recovery_candidate = {
                        'x': new_x,
                        'y': new_y,
                        'yaw': new_yaw
                    }
                    self.stationary_recovery_count = 1

            if self.stationary_recovery_count >= self.stationary_recovery_required_count:
                old_x = self.x
                old_y = self.y
                old_yaw = self.yaw

                pos_alpha = self.stationary_recovery_pos_alpha
                yaw_alpha = self.stationary_recovery_yaw_alpha

                self.x = (1.0 - pos_alpha) * self.x + pos_alpha * new_x
                self.y = (1.0 - pos_alpha) * self.y + pos_alpha * new_y
                self.yaw = self.smooth_angle(self.yaw, new_yaw, yaw_alpha)

                self.pose_initialized_by_aruco = True
                self.control_jump_candidate = None
                self.control_jump_candidate_count = 0
                self.relocalize_candidate = None
                self.relocalize_count = 0

                self.get_logger().warn(
                    f'STATIONARY STABLE RECOVERY APPLIED: '
                    f'old=({old_x:.3f},{old_y:.3f},'
                    f'{math.degrees(old_yaw):.1f}), '
                    f'target=({new_x:.3f},{new_y:.3f},'
                    f'{math.degrees(new_yaw):.1f}), '
                    f'new=({self.x:.3f},{self.y:.3f},'
                    f'{math.degrees(self.yaw):.1f}), '
                    f'dist={jump_dist:.3f}, '
                    f'yaw={math.degrees(jump_yaw):.1f}, '
                    f'markers={marker_count}, '
                    f'reproj={reproj_error:.2f}, '
                    f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                    f'count={self.stationary_recovery_count}'
                )

                return True

            self.aruco_pose_accepted = False
            self.get_logger().warn(
                f'STATIONARY RECOVERY WAIT: '
                f'{self.stationary_recovery_count}/'
                f'{self.stationary_recovery_required_count}, '
                f'dist={jump_dist:.3f}, '
                f'yaw={math.degrees(jump_yaw):.1f}, '
                f'markers={marker_count}, '
                f'reproj={reproj_error:.2f}, '
                f'spread={position_spread:.3f}/{max_position_spread:.3f}'
            )
            return False

        # Permanent recovery rule for real field driving:
        # If only 2~4 markers remain after filtering, do not reject a repeated
        # candidate forever. When the same candidate is stable for several frames,
        # treat it as the robot pose lagging behind the camera result and pull the
        # control pose toward it. This prevents waypoint_drive_node from getting
        # stuck in accepted_lost/stop loops while still rejecting single-marker noise.
        stable_two_marker_allowed = (
            marker_count >= 2
            and marker_count <= 4
            and jump_dist <= self.stable_two_marker_max_jump_m
            and jump_yaw <= math.radians(self.stable_two_marker_max_yaw_deg)
            and reproj_error <= self.stable_two_marker_max_reproj_px
            and max_position_spread <= 0.22
            and max_yaw_spread_deg <= 14.0
        )

        if stable_two_marker_allowed:
            if self.stable_two_marker_candidate is None:
                self.stable_two_marker_candidate = {
                    'x': new_x,
                    'y': new_y,
                    'yaw': new_yaw,
                    'marker_count': marker_count
                }
                self.stable_two_marker_count = 1
            else:
                candidate_dist = math.hypot(
                    new_x - self.stable_two_marker_candidate['x'],
                    new_y - self.stable_two_marker_candidate['y']
                )
                candidate_yaw = abs(
                    self.angle_diff(new_yaw, self.stable_two_marker_candidate['yaw'])
                )
                if (
                    candidate_dist <= self.stable_two_marker_candidate_dist_m
                    and candidate_yaw <= math.radians(self.stable_two_marker_candidate_yaw_deg)
                ):
                    self.stable_two_marker_count += 1
                    self.stable_two_marker_candidate['x'] = new_x
                    self.stable_two_marker_candidate['y'] = new_y
                    self.stable_two_marker_candidate['yaw'] = new_yaw
                    self.stable_two_marker_candidate['marker_count'] = marker_count
                else:
                    self.stable_two_marker_candidate = {
                        'x': new_x,
                        'y': new_y,
                        'yaw': new_yaw,
                        'marker_count': marker_count
                    }
                    self.stable_two_marker_count = 1

            required_count = (
                self.stable_two_marker_required_stopped
                if self.current_cmd == 's'
                else self.stable_two_marker_required_moving
            )

            if self.stable_two_marker_count >= required_count:
                old_x = self.x
                old_y = self.y
                old_yaw = self.yaw

                if self.current_cmd == 's':
                    pos_alpha = self.stable_two_marker_pos_alpha_stopped
                    yaw_alpha = self.stable_two_marker_yaw_alpha_stopped
                    correction_type = 'STABLE 2-MARKER RECOVERY'
                else:
                    pos_alpha = self.stable_two_marker_pos_alpha_moving
                    yaw_alpha = self.stable_two_marker_yaw_alpha_moving
                    correction_type = 'MOVING STABLE 2-MARKER CORRECTION'

                self.x = (1.0 - pos_alpha) * self.x + pos_alpha * new_x
                self.y = (1.0 - pos_alpha) * self.y + pos_alpha * new_y
                self.yaw = self.smooth_angle(self.yaw, new_yaw, yaw_alpha)

                self.pose_initialized_by_aruco = True
                self.control_jump_candidate = None
                self.control_jump_candidate_count = 0
                self.relocalize_candidate = None
                self.relocalize_count = 0
                self.stationary_recovery_candidate = None
                self.stationary_recovery_count = 0

                self.get_logger().warn(
                    f'{correction_type} ACCEPTED: '
                    f'old=({old_x:.3f},{old_y:.3f},'
                    f'{math.degrees(old_yaw):.1f}), '
                    f'candidate=({new_x:.3f},{new_y:.3f},'
                    f'{math.degrees(new_yaw):.1f}), '
                    f'new=({self.x:.3f},{self.y:.3f},'
                    f'{math.degrees(self.yaw):.1f}), '
                    f'dist={jump_dist:.3f}, '
                    f'yaw={math.degrees(jump_yaw):.1f}, '
                    f'markers={marker_count}, '
                    f'reproj={reproj_error:.2f}, '
                    f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                    f'count={self.stable_two_marker_count}, '
                    f'alpha=({pos_alpha:.2f},{yaw_alpha:.2f})'
                )

                return True

            self.aruco_pose_accepted = False
            self.get_logger().warn(
                f'STABLE 2-MARKER RECOVERY WAIT: '
                f'{self.stable_two_marker_count}/{required_count}, '
                f'dist={jump_dist:.3f}, '
                f'yaw={math.degrees(jump_yaw):.1f}, '
                f'markers={marker_count}, '
                f'reproj={reproj_error:.2f}, '
                f'spread={position_spread:.3f}/{max_position_spread:.3f}'
            )
            return False

        self.stable_two_marker_candidate = None
        self.stable_two_marker_count = 0

        self.stationary_recovery_candidate = None
        self.stationary_recovery_count = 0

        if self.current_cmd == 's':
            small_correction_max_dist = 0.25
            small_correction_max_yaw_deg = 30.0
            medium_correction_max_dist = 0.60
            medium_correction_max_yaw_deg = 70.0
            small_pos_alpha = 0.40
            small_yaw_alpha = 0.65
            medium_pos_alpha = 0.60
            medium_yaw_alpha = 0.85
        else:
            small_correction_max_dist = 0.16
            small_correction_max_yaw_deg = 8.0
            medium_correction_max_dist = self.moving_high_conf_max_robust_jump_m
            medium_correction_max_yaw_deg = self.moving_high_conf_max_robust_yaw_deg
            small_pos_alpha = 0.06
            small_yaw_alpha = 0.06
            medium_pos_alpha = self.moving_high_conf_pos_alpha
            medium_yaw_alpha = self.moving_high_conf_yaw_alpha

        small_correction = (
            marker_count >= 5
            and jump_dist <= small_correction_max_dist
            and jump_yaw <= math.radians(small_correction_max_yaw_deg)
            and reproj_error <= self.stationary_recovery_max_reproj_px
            and max_position_spread <= self.stationary_recovery_max_spread_m * 2.0
        )

        medium_correction = (
            marker_count >= self.moving_high_conf_min_markers
            and jump_dist <= medium_correction_max_dist
            and jump_yaw <= math.radians(medium_correction_max_yaw_deg)
            and reproj_error <= self.moving_high_conf_max_reproj_px
            and max_position_spread <= self.stationary_recovery_max_spread_m * 2.5
        )

        if small_correction or medium_correction:
            if small_correction:
                pos_alpha = small_pos_alpha
                yaw_alpha = small_yaw_alpha
                correction_type = 'SMALL'
            else:
                pos_alpha = medium_pos_alpha
                yaw_alpha = medium_yaw_alpha
                correction_type = 'MEDIUM'

            self.control_jump_candidate = None
            self.control_jump_candidate_count = 0
            self.relocalize_candidate = None
            self.relocalize_count = 0

            old_x = self.x
            old_y = self.y
            old_yaw = self.yaw

            self.x = (1.0 - pos_alpha) * self.x + pos_alpha * new_x
            self.y = (1.0 - pos_alpha) * self.y + pos_alpha * new_y
            self.yaw = self.smooth_angle(self.yaw, new_yaw, yaw_alpha)

            self.get_logger().warn(
                f'{correction_type} CONTROL POSE CORRECTION ACCEPTED: '
                f'old=({old_x:.3f},{old_y:.3f},'
                f'{math.degrees(old_yaw):.1f}), '
                f'candidate=({new_x:.3f},{new_y:.3f},'
                f'{math.degrees(new_yaw):.1f}), '
                f'new=({self.x:.3f},{self.y:.3f},'
                f'{math.degrees(self.yaw):.1f}), '
                f'dist={jump_dist:.3f}, '
                f'yaw={math.degrees(jump_yaw):.1f}, '
                f'markers={marker_count}, '
                f'reproj={reproj_error:.2f}, '
                f'spread={position_spread:.3f}/{max_position_spread:.3f}, '
                f'alpha=({pos_alpha:.2f},{yaw_alpha:.2f})'
            )

            return True

        self.control_jump_candidate = None
        self.control_jump_candidate_count = 0
        self.relocalize_candidate = None
        self.relocalize_count = 0

        if (
            self.current_cmd != 's'
            and (jump_dist > 0.50 or jump_yaw > math.radians(15.0) or marker_count <= 3)
        ):
            self.raw_pose_buffer.clear()

        self.get_logger().warn(
            f'CONTROL POSE JUMP REJECTED: '
            f'candidate=({new_x:.3f},{new_y:.3f},'
            f'{math.degrees(new_yaw):.1f}), '
            f'current=({self.x:.3f},{self.y:.3f},'
            f'{math.degrees(self.yaw):.1f}), '
            f'dist={jump_dist:.3f}, '
            f'yaw={math.degrees(jump_yaw):.1f}, '
            f'markers={marker_count}. '
            f'Relaxed jump accept enabled, but this candidate still failed all safety gates.'
        )

        return False

    def get_marker_count_alpha(self, marker_count):
        if marker_count >= 6:
            return 0.55, 0.45
        if marker_count == 5:
            return 0.45, 0.40
        if marker_count == 4:
            return 0.35, 0.32
        if marker_count == 3:
            return 0.25, 0.25
        if marker_count == 2:
            return 0.08, 0.10
        if marker_count == 1:
            return 0.00, 0.00

        return 0.00, 0.00

    def apply_aruco_pose_with_confidence(
        self,
        robot_x,
        robot_y,
        robot_yaw,
        marker_count,
        main_marker_id,
        single_rb_valid=False
    ):
        pos_alpha, yaw_alpha = self.get_marker_count_alpha(marker_count)

        if self.aruco_transform['use_smoothing']:
            config_alpha = self.aruco_transform['alpha']
            pos_alpha = min(pos_alpha, config_alpha)
            yaw_alpha = min(yaw_alpha, config_alpha)
        else:
            pos_alpha = min(pos_alpha, 1.0)
            yaw_alpha = min(yaw_alpha, 1.0)

        if not self.pose_initialized_by_aruco:
            if marker_count >= 3:
                self.x = robot_x
                self.y = robot_y
                self.yaw = robot_yaw
                self.pose_initialized_by_aruco = True
                self.init_candidate = None
                self.init_count = 0
                self.relocalize_candidate = None
                self.relocalize_count = 0
                self.control_jump_candidate = None
                self.control_jump_candidate_count = 0
                self.stationary_recovery_candidate = None
                self.stationary_recovery_count = 0
                self.stable_two_marker_candidate = None
                self.stable_two_marker_count = 0

                self.apply_pending_face_lock(marker_count)

                self.get_logger().warn(
                    f'Initial ArUco pose set by stable multi marker: '
                    f'x={self.x:.3f}, y={self.y:.3f}, '
                    f'yaw={math.degrees(self.yaw):.1f}, markers={marker_count}'
                )
                return True

            if marker_count == 2:
                if self.init_candidate is None:
                    self.init_candidate = {
                        'x': robot_x,
                        'y': robot_y,
                        'yaw': robot_yaw,
                        'marker_id': main_marker_id,
                        'marker_count': marker_count
                    }
                    self.init_count = 1
                else:
                    candidate_dist = math.hypot(
                        robot_x - self.init_candidate['x'],
                        robot_y - self.init_candidate['y']
                    )
                    candidate_yaw = abs(
                        self.angle_diff(robot_yaw, self.init_candidate['yaw'])
                    )

                    if (
                        candidate_dist <= 0.08
                        and candidate_yaw <= math.radians(8.0)
                    ):
                        self.init_count += 1
                        self.init_candidate['x'] = robot_x
                        self.init_candidate['y'] = robot_y
                        self.init_candidate['yaw'] = robot_yaw
                        self.init_candidate['marker_id'] = main_marker_id
                        self.init_candidate['marker_count'] = marker_count
                    else:
                        self.init_candidate = {
                            'x': robot_x,
                            'y': robot_y,
                            'yaw': robot_yaw,
                            'marker_id': main_marker_id,
                            'marker_count': marker_count
                        }
                        self.init_count = 1

                required_init_count = 4

                if self.init_count >= required_init_count:
                    self.x = robot_x
                    self.y = robot_y
                    self.yaw = robot_yaw
                    self.pose_initialized_by_aruco = True
                    self.init_candidate = None
                    self.init_count = 0
                    self.relocalize_candidate = None
                    self.relocalize_count = 0
                    self.control_jump_candidate = None
                    self.control_jump_candidate_count = 0
                    self.stationary_recovery_candidate = None
                    self.stationary_recovery_count = 0
                    self.stable_two_marker_candidate = None
                    self.stable_two_marker_count = 0
                    self.apply_pending_face_lock(marker_count)

                    self.get_logger().warn(
                        f'Initial ArUco pose set by stable 2-marker: '
                        f'x={self.x:.3f}, y={self.y:.3f}, '
                        f'yaw={math.degrees(self.yaw):.1f}, '
                        f'markers={marker_count}'
                    )
                    return True

                self.get_logger().warn(
                    f'Initial 2-marker pose waiting: '
                    f'{self.init_count}/{required_init_count}, '
                    f'x={robot_x:.3f}, y={robot_y:.3f}, '
                    f'yaw={math.degrees(robot_yaw):.1f}, '
                    f'markers={marker_count}, marker_id={main_marker_id}'
                )
                return False

            self.get_logger().warn(
                f'Initial ArUco pose waiting: need 2+ markers, '
                f'current={marker_count}'
            )
            self.init_candidate = None
            self.init_count = 0
            return False

        pos_error = math.hypot(robot_x - self.x, robot_y - self.y)
        yaw_error = abs(self.angle_diff(robot_yaw, self.yaw))

        if (
            pos_error > self.relocalize_big_jump_m
            and marker_count >= self.relocalize_min_marker_count
        ):
            return self.try_relocalize(
                robot_x,
                robot_y,
                robot_yaw,
                marker_count,
                pos_error
            )

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

        accepted = pos_alpha > 0.0 or yaw_alpha > 0.0

        if not accepted:
            self.get_logger().warn(
                f'ArUco pose detected but not applied: '
                f'markers={marker_count}, pos_error={pos_error:.3f}, '
                f'yaw_error={math.degrees(yaw_error):.1f}deg'
            )
            return False

        self.x = (1.0 - pos_alpha) * self.x + pos_alpha * robot_x
        self.y = (1.0 - pos_alpha) * self.y + pos_alpha * robot_y
        self.yaw = self.smooth_angle(self.yaw, robot_yaw, yaw_alpha)

        return True

    def try_single_marker_initialization(self, robot_x, robot_y, robot_yaw, marker_id):
        if marker_id < 0:
            return False

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
            return True

        return False

    def try_relocalize(
        self,
        robot_x,
        robot_y,
        robot_yaw,
        marker_count,
        pos_error
    ):
        yaw_error = abs(
            self.angle_diff(
                robot_yaw,
                self.yaw
            )
        )

        self.relocalize_candidate = None
        self.relocalize_count = 0

        if (
            self.current_cmd != 's'
            and (pos_error > 0.80 or yaw_error > math.radians(20.0) or marker_count <= 3)
        ):
            self.raw_pose_buffer.clear()

        self.get_logger().warn(
            f'FORCE RELOCALIZE REJECTED: '
            f'raw=({robot_x:.3f},{robot_y:.3f},'
            f'{math.degrees(robot_yaw):.1f}), '
            f'current=({self.x:.3f},{self.y:.3f},'
            f'{math.degrees(self.yaw):.1f}), '
            f'pos_error={pos_error:.3f}, '
            f'yaw_error={math.degrees(yaw_error):.1f}, '
            f'markers={marker_count}. '
            f'Moving large relocalize disabled.'
        )

        return False

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
            self.raw_pose_buffer.clear()
            self.control_jump_candidate = None
            self.control_jump_candidate_count = 0
            self.stationary_recovery_candidate = None
            self.stationary_recovery_count = 0

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

        if self.aruco_pose_accepted:
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
        reproj_error=0.0,
        marker_count_used=0,
        used_ids_text='',
        pnp_label='',
        active_face=''
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
            f'used_ids={used_ids_text},'
            f'marker_count_used={marker_count_used},'
            f'pnp_label={pnp_label},'
            f'active_face={active_face},'
            f'marker_seen={1 if self.marker_seen else 0},'
            f'aruco_accepted={1 if self.aruco_pose_accepted else 0},'
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
            f'robust_x={self.last_robust_x:.3f},'
            f'robust_y={self.last_robust_y:.3f},'
            f'robust_yaw={self.last_robust_yaw:.4f},'
            f'robust_yaw_deg={math.degrees(self.last_robust_yaw):.2f},'
            f'raw_buffer_count={len(self.raw_pose_buffer)},'
            f'control_jump_count={self.control_jump_candidate_count},'
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

        if self.should_log_pose(cmd):
            self.get_logger().info(out)

    def should_log_pose(self, cmd):
        if not self.pose_log_enabled:
            return False

        now = self.get_clock().now()

        key = (
            self.last_source,
            int(self.marker_seen),
            int(self.aruco_pose_accepted),
            self.last_marker_id,
            str(cmd)
        )

        if key != self.last_pose_log_key:
            self.last_pose_log_key = key
            self.last_pose_log_time = now
            return True

        if self.last_pose_log_time is None:
            self.last_pose_log_time = now
            return True

        elapsed = (now - self.last_pose_log_time).nanoseconds / 1e9

        if elapsed >= self.pose_log_interval_sec:
            self.last_pose_log_time = now
            return True

        return False



def main(args=None):
    rclpy.init(args=args)

    node = RelativePoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
