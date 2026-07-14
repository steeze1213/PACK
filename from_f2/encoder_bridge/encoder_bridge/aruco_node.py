#!/usr/bin/env python3
import cv2
import time
import math
import yaml
import os
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.marker_pub = self.create_publisher(
            String,
            '/aruco_marker',
            10
        )

        self.multi_marker_pub = self.create_publisher(
            String,
            '/aruco_multi_markers',
            10
        )

        self.target_marker_pub = self.create_publisher(
            String,
            '/target_marker',
            10
        )

        self.target_marker_cmd_sub = self.create_subscription(
            String,
            '/target_marker_cmd',
            self.target_marker_cmd_callback,
            10
        )

        self.image_pub = self.create_publisher(
            CompressedImage,
            '/aruco_image/compressed',
            10
        )

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('static_marker_ids', list(range(10, 142)))
        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('marker_size_m', 0.050)
        self.declare_parameter('target_marker_size_m', 0.020)
        self.declare_parameter(
            'calib_path',
            '~/robot_ws/src/encoder_bridge/config/camera_calibration.yaml'
        )

        self.camera_index = int(
            self.get_parameter('camera_index').value
        )
        self.frame_width = int(
            self.get_parameter('frame_width').value
        )
        self.frame_height = int(
            self.get_parameter('frame_height').value
        )

        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2

        self.static_marker_ids = [
            int(marker_id)
            for marker_id in self.get_parameter(
                'static_marker_ids'
            ).value
        ]

        self.target_marker_id = int(
            self.get_parameter('target_marker_id').value
        )

        self.marker_size_m = float(
            self.get_parameter('marker_size_m').value
        )

        self.target_marker_size_m = float(
            self.get_parameter('target_marker_size_m').value
        )

        self.calib_path = os.path.expanduser(
            str(self.get_parameter('calib_path').value)
        )

        self.camera_matrix = None
        self.dist_coeffs = None

        self.load_camera_calibration()

        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            self.get_logger().error('Camera open failed')
            raise RuntimeError('Camera open failed')

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.frame_width
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.frame_height
        )

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        self.aruco_dict = self.get_aruco_dictionary()
        self.parameters = self.get_detector_parameters()

        self.clahe = cv2.createCLAHE(
            clipLimit=2.5,
            tileGridSize=(8, 8)
        )

        self.last_detect_mode = 'NONE'
        self.last_detected_static_ids = []

        self.no_static_marker_count = 0
        self.no_target_marker_count = 0

        self.min_marker_width_px = 7.0
        self.max_marker_width_px = 420.0
        self.min_marker_area_px = 50.0
        self.max_marker_aspect_ratio = 2.60
        self.min_marker_aspect_ratio = 0.38
        self.duplicate_log_period_sec = 1.0
        self.last_duplicate_log_time = 0.0
        self.last_filter_log_time = 0.0

        self.border_margin_px = 10.0
        self.max_bearing_yaw_deg = 65.0
        self.max_static_distance_m = 4.00
        self.max_offset_y_px = 230.0

        time.sleep(1.0)

        self.timer = self.create_timer(
            0.1,
            self.detect_marker
        )

        self.get_logger().info(
            'Aruco node started: tiny-marker clean-publish static localization markers'
        )

        self.get_logger().info(
            f'Static marker IDs: {self.static_marker_ids}'
        )

        self.get_logger().info(
            f'Initial target marker ID: {self.target_marker_id}'
        )

        self.get_logger().info(
            'Target marker command topic: /target_marker_cmd'
        )

        self.get_logger().info(
            f'Static marker size: {self.marker_size_m} m'
        )

        self.get_logger().info(
            f'Target marker size: {self.target_marker_size_m} m'
        )

        self.get_logger().info(
            'Detection order: NORMAL -> CLAHE -> SHARP'
        )

        self.get_logger().info(
            'Duplicate marker IDs are removed before publishing multi markers'
        )

        self.get_logger().info(
            'Candidate geometry filter enabled'
        )

        self.get_logger().info(
            f'Pose publish filter: min_width={self.min_marker_width_px:.1f}px, '
            f'min_area={self.min_marker_area_px:.1f}px, '
            f'aspect=[{self.min_marker_aspect_ratio:.2f}, {self.max_marker_aspect_ratio:.2f}], '
            f'border_margin={self.border_margin_px:.0f}px'
        )

    def target_marker_cmd_callback(self, msg):
        raw = msg.data.strip()

        try:
            marker_id = int(raw)
        except ValueError:
            self.get_logger().warn(
                f'Invalid target_marker_cmd ignored: {raw}'
            )
            return

        if marker_id < 0:
            self.get_logger().warn(
                f'Invalid target_marker_cmd ignored: {marker_id}'
            )
            return

        old_id = self.target_marker_id
        self.target_marker_id = marker_id
        self.no_target_marker_count = 0

        self.get_logger().warn(
            f'Target marker changed: {old_id} -> {self.target_marker_id}'
        )

    def get_aruco_dictionary(self):
        try:
            return cv2.aruco.Dictionary_get(
                cv2.aruco.DICT_5X5_250
            )
        except AttributeError:
            return cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_5X5_250
            )

    def get_detector_parameters(self):
        try:
            params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            params = cv2.aruco.DetectorParameters()

        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 45
        params.adaptiveThreshWinSizeStep = 4
        params.adaptiveThreshConstant = 7

        params.minMarkerPerimeterRate = 0.010
        params.maxMarkerPerimeterRate = 4.0

        params.polygonalApproxAccuracyRate = 0.07
        params.minCornerDistanceRate = 0.02
        params.minMarkerDistanceRate = 0.02

        params.minDistanceToBorder = 1
        params.minOtsuStdDev = 3.0

        params.perspectiveRemovePixelPerCell = 8
        params.perspectiveRemoveIgnoredMarginPerCell = 0.10

        params.maxErroneousBitsInBorderRate = 0.45
        params.errorCorrectionRate = 0.60

        try:
            params.cornerRefinementMethod = (
                cv2.aruco.CORNER_REFINE_SUBPIX
            )

            params.cornerRefinementWinSize = 5
            params.cornerRefinementMaxIterations = 40
            params.cornerRefinementMinAccuracy = 0.05
        except AttributeError:
            pass

        return params

    def load_camera_calibration(self):
        if not os.path.exists(self.calib_path):
            raise RuntimeError(
                f'Calibration file not found: {self.calib_path}'
            )

        with open(self.calib_path, 'r') as f:
            data = yaml.safe_load(f)

        camera_data = data['camera_matrix']['data']
        dist_data = data['distortion_coefficients']['data']

        self.camera_matrix = np.array(
            camera_data,
            dtype=np.float64
        ).reshape(3, 3)

        self.dist_coeffs = np.array(
            dist_data,
            dtype=np.float64
        ).reshape(1, -1)

    def detect_marker(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Frame read failed')
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected, detect_mode = (
            self.detect_markers_multi_stage(gray)
        )

        self.last_detect_mode = detect_mode

        static_candidates = []
        target_candidate = None

        if ids is not None and len(ids) > 0:
            marker_ids = ids.flatten()

            cv2.aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            for i, raw_id in enumerate(marker_ids):
                marker_id = int(raw_id)

                if (
                    marker_id not in self.static_marker_ids
                    and marker_id != self.target_marker_id
                ):
                    continue

                marker_corner = corners[i]

                if marker_id == self.target_marker_id:
                    marker_size = self.target_marker_size_m
                else:
                    marker_size = self.marker_size_m

                candidate = self.create_marker_candidate(
                    marker_id,
                    marker_corner,
                    marker_size
                )

                if candidate is None:
                    continue

                if not self.is_candidate_valid(candidate):
                    continue

                if marker_id == self.target_marker_id:
                    if (
                        target_candidate is None
                        or candidate['score'] < target_candidate['score']
                    ):
                        target_candidate = candidate
                else:
                    static_candidates.append(candidate)

            static_candidates = self.deduplicate_static_candidates(
                static_candidates
            )

        if len(static_candidates) > 0:
            selected_static = min(
                static_candidates,
                key=lambda candidate: candidate['score']
            )

            self.no_static_marker_count = 0

            self.last_detected_static_ids = [
                candidate['marker_id']
                for candidate in static_candidates
            ]

            self.publish_multi_markers(
                static_candidates
            )

            self.publish_selected_static_marker(
                frame,
                selected_static,
                static_candidates,
                detect_mode
            )
        else:
            self.no_static_marker_count += 1
            self.last_detected_static_ids = []

            self.publish_no_static_marker(
                detect_mode
            )

        if target_candidate is not None:
            self.no_target_marker_count = 0

            self.publish_target_marker(
                frame,
                target_candidate,
                detect_mode
            )
        else:
            self.no_target_marker_count += 1

            self.publish_no_target_marker(
                detect_mode
            )

        if (
            len(static_candidates) == 0
            and target_candidate is None
        ):
            self.draw_rejected_candidates(
                frame,
                rejected
            )

            self.draw_no_marker_info(
                frame,
                detect_mode,
                len(rejected)
            )
        else:
            self.draw_detection_summary(
                frame,
                static_candidates,
                target_candidate,
                detect_mode
            )

        self.draw_center_lines(frame)
        self.publish_image(frame)

    def deduplicate_static_candidates(self, candidates):
        if len(candidates) <= 1:
            return candidates

        best_by_id = {}
        duplicate_ids = {}

        for candidate in candidates:
            marker_id = candidate['marker_id']

            if marker_id in best_by_id:
                duplicate_ids[marker_id] = duplicate_ids.get(
                    marker_id,
                    1
                ) + 1

                if candidate['score'] < best_by_id[marker_id]['score']:
                    best_by_id[marker_id] = candidate
            else:
                best_by_id[marker_id] = candidate

        unique_candidates = list(best_by_id.values())
        unique_candidates.sort(
            key=lambda candidate: candidate['marker_id']
        )

        if len(duplicate_ids) > 0:
            now = time.time()

            if (
                now - self.last_duplicate_log_time
                >= self.duplicate_log_period_sec
            ):
                duplicate_text = ','.join(
                    [
                        f'{marker_id}x{count}'
                        for marker_id, count in sorted(
                            duplicate_ids.items()
                        )
                    ]
                )

                self.get_logger().warn(
                    f'Duplicate ArUco IDs removed: '
                    f'{duplicate_text}, '
                    f'raw_count={len(candidates)}, '
                    f'unique_count={len(unique_candidates)}'
                )

                self.last_duplicate_log_time = now

        return unique_candidates

    def is_candidate_valid(self, candidate):
        marker_id = candidate['marker_id']
        width = float(candidate['marker_width_px'])
        area = float(candidate['area'])
        rel_z = float(candidate['rel_z'])

        if width < self.min_marker_width_px:
            self.log_candidate_filtered(
                marker_id,
                f'width too small {width:.1f}px'
            )
            return False

        if width > self.max_marker_width_px:
            self.log_candidate_filtered(
                marker_id,
                f'width too large {width:.1f}px'
            )
            return False

        if area < self.min_marker_area_px:
            self.log_candidate_filtered(
                marker_id,
                f'area too small {area:.1f}px'
            )
            return False

        if rel_z <= 0.0:
            self.log_candidate_filtered(
                marker_id,
                f'rel_z invalid {rel_z:.3f}'
            )
            return False

        if candidate['distance_3d'] > self.max_static_distance_m:
            self.log_candidate_filtered(
                marker_id,
                f'distance too far {candidate["distance_3d"]:.2f}m'
            )
            return False

        if abs(candidate['bearing_yaw_deg']) > self.max_bearing_yaw_deg:
            self.log_candidate_filtered(
                marker_id,
                f'bearing too large {candidate["bearing_yaw_deg"]:.1f}deg'
            )
            return False

        if abs(candidate['offset_y']) > self.max_offset_y_px:
            self.log_candidate_filtered(
                marker_id,
                f'vertical offset too large {candidate["offset_y"]:.1f}px'
            )
            return False

        points = candidate['corner'][0]

        min_x = min(float(p[0]) for p in points)
        max_x = max(float(p[0]) for p in points)
        min_y = min(float(p[1]) for p in points)
        max_y = max(float(p[1]) for p in points)

        if (
            min_x < self.border_margin_px
            or max_x > self.frame_width - self.border_margin_px
            or min_y < self.border_margin_px
            or max_y > self.frame_height - self.border_margin_px
        ):
            self.log_candidate_filtered(
                marker_id,
                f'near image border box=({min_x:.0f},{min_y:.0f})-({max_x:.0f},{max_y:.0f})'
            )
            return False

        side_lengths = []

        for idx in range(4):
            p1 = points[idx]
            p2 = points[(idx + 1) % 4]

            side = math.sqrt(
                float(p2[0] - p1[0]) ** 2
                + float(p2[1] - p1[1]) ** 2
            )

            side_lengths.append(side)

        min_side = max(min(side_lengths), 1e-6)
        max_side = max(side_lengths)
        aspect = max_side / min_side

        if aspect > self.max_marker_aspect_ratio:
            self.log_candidate_filtered(
                marker_id,
                f'aspect too large {aspect:.2f}'
            )
            return False

        if aspect < self.min_marker_aspect_ratio:
            self.log_candidate_filtered(
                marker_id,
                f'aspect too small {aspect:.2f}'
            )
            return False

        return True

    def log_candidate_filtered(self, marker_id, reason):
        now = time.time()

        if now - self.last_filter_log_time < 1.0:
            return

        self.get_logger().warn(
            f'Filtered ArUco candidate ID {marker_id}: {reason}'
        )

        self.last_filter_log_time = now

    def create_marker_candidate(
        self,
        marker_id,
        marker_corner,
        marker_size
    ):
        cx, cy = self.calculate_marker_center(
            marker_corner
        )

        area = self.calculate_marker_area(
            marker_corner
        )

        marker_width_px = self.calculate_marker_width(
            marker_corner
        )

        offset_x = cx - self.center_x
        offset_y = cy - self.center_y

        try:
            rvecs, tvecs, _ = (
                cv2.aruco.estimatePoseSingleMarkers(
                    [marker_corner],
                    marker_size,
                    self.camera_matrix,
                    self.dist_coeffs
                )
            )
        except cv2.error as e:
            self.get_logger().warn(
                f'Pose estimation failed for ID {marker_id}: {e}'
            )
            return None

        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        rel_x = float(tvec[0])
        rel_y = float(tvec[1])
        rel_z = float(tvec[2])

        if rel_z <= 0.0:
            return None

        planar_distance = math.sqrt(
            rel_x * rel_x
            + rel_z * rel_z
        )

        distance_3d = math.sqrt(
            rel_x * rel_x
            + rel_y * rel_y
            + rel_z * rel_z
        )

        bearing_yaw = math.atan2(
            rel_x,
            rel_z
        )

        bearing_yaw_deg = math.degrees(
            bearing_yaw
        )

        marker_roll, marker_pitch, marker_yaw = (
            self.rvec_to_euler(rvec)
        )

        points = marker_corner[0]
        side_lengths = []

        for idx in range(4):
            p1 = points[idx]
            p2 = points[(idx + 1) % 4]
            side_lengths.append(
                math.hypot(
                    float(p2[0] - p1[0]),
                    float(p2[1] - p1[1])
                )
            )

        min_side = max(min(side_lengths), 1e-6)
        max_side = max(side_lengths)
        aspect = max_side / min_side
        aspect_error = abs(aspect - 1.0)

        min_x = min(float(p[0]) for p in points)
        max_x = max(float(p[0]) for p in points)
        min_y = min(float(p[1]) for p in points)
        max_y = max(float(p[1]) for p in points)
        edge_distance = min(
            min_x,
            min_y,
            self.frame_width - max_x,
            self.frame_height - max_y
        )

        score = (
            abs(offset_x) * 0.50
            + abs(offset_y) * 0.20
            + abs(bearing_yaw_deg) * 3.0
            + rel_z * 8.0
            + aspect_error * 120.0
            - marker_width_px * 0.35
            - min(edge_distance, 80.0) * 0.20
        )

        return {
            'marker_id': marker_id,
            'corner': marker_corner,
            'cx': cx,
            'cy': cy,
            'area': area,
            'marker_width_px': marker_width_px,
            'offset_x': offset_x,
            'offset_y': offset_y,
            'rvec': rvec,
            'tvec': tvec,
            'rel_x': rel_x,
            'rel_y': rel_y,
            'rel_z': rel_z,
            'planar_distance': planar_distance,
            'distance_3d': distance_3d,
            'bearing_yaw': bearing_yaw,
            'bearing_yaw_deg': bearing_yaw_deg,
            'marker_roll': marker_roll,
            'marker_pitch': marker_pitch,
            'marker_yaw': marker_yaw,
            'aspect': aspect,
            'aspect_error': aspect_error,
            'edge_distance': edge_distance,
            'score': score
        }

    def detect_markers_multi_stage(self, gray):
        corners, ids, rejected = self.run_marker_detection(
            gray
        )

        if ids is not None and len(ids) > 0:
            return corners, ids, rejected, 'NORMAL'

        best_rejected = rejected

        clahe_gray = self.clahe.apply(gray)

        corners, ids, rejected_clahe = (
            self.run_marker_detection(clahe_gray)
        )

        if len(rejected_clahe) > len(best_rejected):
            best_rejected = rejected_clahe

        if ids is not None and len(ids) > 0:
            return corners, ids, rejected_clahe, 'CLAHE'

        blurred = cv2.GaussianBlur(
            clahe_gray,
            (0, 0),
            1.2
        )

        sharp_gray = cv2.addWeighted(
            clahe_gray,
            1.8,
            blurred,
            -0.8,
            0
        )

        corners, ids, rejected_sharp = (
            self.run_marker_detection(sharp_gray)
        )

        if len(rejected_sharp) > len(best_rejected):
            best_rejected = rejected_sharp

        if ids is not None and len(ids) > 0:
            return corners, ids, rejected_sharp, 'SHARP'

        return [], None, best_rejected, 'FAILED'

    def run_marker_detection(self, image):
        try:
            corners, ids, rejected = (
                cv2.aruco.detectMarkers(
                    image,
                    self.aruco_dict,
                    parameters=self.parameters
                )
            )
        except cv2.error as e:
            self.get_logger().warn(
                f'detectMarkers error: {e}'
            )

            return [], None, []

        if rejected is None:
            rejected = []

        return corners, ids, rejected

    def publish_multi_markers(self, static_candidates):
        static_candidates = self.deduplicate_static_candidates(
            static_candidates
        )

        static_candidates.sort(
            key=lambda candidate: candidate['marker_id']
        )

        parts = [
            'MULTI_ARUCO',
            'detected=1',
            f'count={len(static_candidates)}',
            f'unique_count={len(static_candidates)}'
        ]

        for idx, candidate in enumerate(static_candidates):
            points = candidate['corner'][0]

            parts.append(
                f'm{idx}_id={candidate["marker_id"]}'
            )

            for corner_idx in range(4):
                x = float(
                    points[corner_idx][0]
                )

                y = float(
                    points[corner_idx][1]
                )

                parts.append(
                    f'm{idx}_c{corner_idx}x={x:.3f}'
                )

                parts.append(
                    f'm{idx}_c{corner_idx}y={y:.3f}'
                )

        msg = String()
        msg.data = ','.join(parts)

        self.multi_marker_pub.publish(msg)

    def publish_no_static_marker(self, detect_mode):
        marker_msg = String()

        marker_msg.data = (
            'ARUCO,'
            'id=-1,'
            'detected=0,'
            f'detect_mode={detect_mode},'
            'cx=-1,'
            'cy=-1,'
            'offset_x=0,'
            'offset_y=0,'
            'area=0,'
            'marker_width_px=0.0,'
            'rel_x=0.000,'
            'rel_y=0.000,'
            'rel_z=0.000,'
            'bearing_yaw=0.0000,'
            'bearing_yaw_deg=0.00,'
            'marker_roll=0.0000,'
            'marker_pitch=0.0000,'
            'marker_yaw=0.0000,'
            'rvec_x=0.000000,'
            'rvec_y=0.000000,'
            'rvec_z=0.000000,'
            'tvec_x=0.000000,'
            'tvec_y=0.000000,'
            'tvec_z=0.000000'
        )

        self.marker_pub.publish(marker_msg)

        multi_msg = String()
        multi_msg.data = (
            'MULTI_ARUCO,'
            'detected=0,'
            'count=0'
        )

        self.multi_marker_pub.publish(multi_msg)

    def publish_selected_static_marker(
        self,
        frame,
        selected,
        static_candidates,
        detect_mode
    ):
        marker_id = selected['marker_id']
        cx = selected['cx']
        cy = selected['cy']
        offset_x = selected['offset_x']
        offset_y = selected['offset_y']
        area = selected['area']
        marker_width_px = selected['marker_width_px']
        rel_x = selected['rel_x']
        rel_y = selected['rel_y']
        rel_z = selected['rel_z']
        bearing_yaw = selected['bearing_yaw']
        bearing_yaw_deg = selected['bearing_yaw_deg']
        marker_roll = selected['marker_roll']
        marker_pitch = selected['marker_pitch']
        marker_yaw = selected['marker_yaw']
        rvec = selected['rvec']
        tvec = selected['tvec']

        marker_msg = String()

        marker_msg.data = (
            f'ARUCO,'
            f'id={marker_id},'
            f'detected=1,'
            f'detect_mode={detect_mode},'
            f'cx={cx},'
            f'cy={cy},'
            f'offset_x={offset_x},'
            f'offset_y={offset_y},'
            f'area={int(area)},'
            f'marker_width_px={marker_width_px:.1f},'
            f'rel_x={rel_x:.3f},'
            f'rel_y={rel_y:.3f},'
            f'rel_z={rel_z:.3f},'
            f'bearing_yaw={bearing_yaw:.4f},'
            f'bearing_yaw_deg={bearing_yaw_deg:.2f},'
            f'marker_roll={marker_roll:.4f},'
            f'marker_pitch={marker_pitch:.4f},'
            f'marker_yaw={marker_yaw:.4f},'
            f'rvec_x={float(rvec[0]):.6f},'
            f'rvec_y={float(rvec[1]):.6f},'
            f'rvec_z={float(rvec[2]):.6f},'
            f'tvec_x={float(tvec[0]):.6f},'
            f'tvec_y={float(tvec[1]):.6f},'
            f'tvec_z={float(tvec[2]):.6f}'
        )

        self.marker_pub.publish(marker_msg)

        self.draw_static_marker_info(
            frame,
            selected,
            static_candidates,
            detect_mode
        )

        try:
            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_size_m * 0.5
            )
        except (AttributeError, cv2.error):
            pass

    def publish_target_marker(
        self,
        frame,
        target,
        detect_mode
    ):
        marker_id = target['marker_id']
        cx = target['cx']
        cy = target['cy']
        offset_x = target['offset_x']
        offset_y = target['offset_y']
        area = target['area']
        marker_width_px = target['marker_width_px']
        rel_x = target['rel_x']
        rel_y = target['rel_y']
        rel_z = target['rel_z']
        planar_distance = target['planar_distance']
        distance_3d = target['distance_3d']
        bearing_yaw = target['bearing_yaw']
        bearing_yaw_deg = target['bearing_yaw_deg']
        marker_roll = target['marker_roll']
        marker_pitch = target['marker_pitch']
        marker_yaw = target['marker_yaw']
        rvec = target['rvec']
        tvec = target['tvec']

        target_msg = String()

        target_msg.data = (
            f'TARGET_MARKER,'
            f'id={marker_id},'
            f'detected=1,'
            f'detect_mode={detect_mode},'
            f'cx={cx},'
            f'cy={cy},'
            f'offset_x={offset_x},'
            f'offset_y={offset_y},'
            f'area={int(area)},'
            f'marker_width_px={marker_width_px:.1f},'
            f'rel_x={rel_x:.3f},'
            f'rel_y={rel_y:.3f},'
            f'rel_z={rel_z:.3f},'
            f'planar_distance={planar_distance:.3f},'
            f'distance_3d={distance_3d:.3f},'
            f'bearing_yaw={bearing_yaw:.4f},'
            f'bearing_yaw_deg={bearing_yaw_deg:.2f},'
            f'marker_roll={marker_roll:.4f},'
            f'marker_pitch={marker_pitch:.4f},'
            f'marker_yaw={marker_yaw:.4f},'
            f'rvec_x={float(rvec[0]):.6f},'
            f'rvec_y={float(rvec[1]):.6f},'
            f'rvec_z={float(rvec[2]):.6f},'
            f'tvec_x={float(tvec[0]):.6f},'
            f'tvec_y={float(tvec[1]):.6f},'
            f'tvec_z={float(tvec[2]):.6f}'
        )

        self.target_marker_pub.publish(target_msg)

        self.draw_target_marker_info(
            frame,
            target,
            detect_mode
        )

        try:
            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.target_marker_size_m * 0.5
            )
        except (AttributeError, cv2.error):
            pass

    def publish_no_target_marker(self, detect_mode):
        target_msg = String()

        target_msg.data = (
            'TARGET_MARKER,'
            f'id={self.target_marker_id},'
            'detected=0,'
            f'detect_mode={detect_mode},'
            'cx=-1,'
            'cy=-1,'
            'offset_x=0,'
            'offset_y=0,'
            'area=0,'
            'marker_width_px=0.0,'
            'rel_x=0.000,'
            'rel_y=0.000,'
            'rel_z=0.000,'
            'planar_distance=0.000,'
            'distance_3d=0.000,'
            'bearing_yaw=0.0000,'
            'bearing_yaw_deg=0.00,'
            'marker_roll=0.0000,'
            'marker_pitch=0.0000,'
            'marker_yaw=0.0000,'
            'rvec_x=0.000000,'
            'rvec_y=0.000000,'
            'rvec_z=0.000000,'
            'tvec_x=0.000000,'
            'tvec_y=0.000000,'
            'tvec_z=0.000000'
        )

        self.target_marker_pub.publish(target_msg)

    def draw_static_marker_info(
        self,
        frame,
        selected,
        static_candidates,
        detect_mode
    ):
        marker_id = selected['marker_id']
        cx = selected['cx']
        cy = selected['cy']
        offset_x = selected['offset_x']
        rel_z = selected['rel_z']

        cv2.circle(
            frame,
            (cx, cy),
            7,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            frame,
            f'STATIC ID {marker_id} '
            f'/ count={len(static_candidates)}',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            f'STATIC offset={offset_x} '
            f'z={rel_z:.2f}m',
            (20, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            f'DETECT MODE: {detect_mode}',
            (20, 86),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0),
            2
        )

    def draw_target_marker_info(
        self,
        frame,
        target,
        detect_mode
    ):
        cx = target['cx']
        cy = target['cy']
        offset_x = target['offset_x']
        rel_x = target['rel_x']
        rel_z = target['rel_z']
        distance = target['planar_distance']
        bearing = target['bearing_yaw_deg']

        cv2.circle(
            frame,
            (cx, cy),
            9,
            (0, 255, 0),
            -1
        )

        cv2.putText(
            frame,
            f'TARGET ID {self.target_marker_id}',
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f'distance={distance:.3f}m '
            f'bearing={bearing:.1f}deg',
            (20, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f'rel_x={rel_x:.3f} '
            f'rel_z={rel_z:.3f} '
            f'offset={offset_x}',
            (20, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f'TARGET DETECT: {detect_mode}',
            (20, 210),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0),
            2
        )

    def draw_detection_summary(
        self,
        frame,
        static_candidates,
        target_candidate,
        detect_mode
    ):
        target_status = (
            'YES'
            if target_candidate is not None
            else 'NO'
        )

        cv2.putText(
            frame,
            f'STATIC={len(static_candidates)} '
            f'TARGET{self.target_marker_id}={target_status} '
            f'MODE={detect_mode}',
            (20, 455),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2
        )

    def draw_no_marker_info(
        self,
        frame,
        detect_mode,
        rejected_count
    ):
        cv2.putText(
            frame,
            'No static marker and no target marker',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            f'DETECT MODE: {detect_mode}',
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 165, 255),
            2
        )

        cv2.putText(
            frame,
            f'NO STATIC COUNT: {self.no_static_marker_count}',
            (20, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 165, 255),
            2
        )

        cv2.putText(
            frame,
            f'NO TARGET COUNT: {self.no_target_marker_count}',
            (20, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 165, 255),
            2
        )

        cv2.putText(
            frame,
            f'REJECTED: {rejected_count}',
            (20, 190),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 0, 255),
            2
        )

    def draw_rejected_candidates(self, frame, rejected):
        if rejected is None:
            return

        for candidate in rejected[:30]:
            try:
                points = candidate.reshape(
                    4,
                    2
                ).astype(np.int32)

                cv2.polylines(
                    frame,
                    [points],
                    True,
                    (255, 0, 255),
                    1
                )
            except Exception:
                continue

        cv2.putText(
            frame,
            f'REJECTED CANDIDATES: {len(rejected)}',
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 0, 255),
            2
        )

    def rvec_to_euler(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(
            rvec
        )

        sy = math.sqrt(
            rotation_matrix[0, 0]
            * rotation_matrix[0, 0]
            + rotation_matrix[1, 0]
            * rotation_matrix[1, 0]
        )

        singular = sy < 1e-6

        if not singular:
            roll = math.atan2(
                rotation_matrix[2, 1],
                rotation_matrix[2, 2]
            )

            pitch = math.atan2(
                -rotation_matrix[2, 0],
                sy
            )

            yaw = math.atan2(
                rotation_matrix[1, 0],
                rotation_matrix[0, 0]
            )
        else:
            roll = math.atan2(
                -rotation_matrix[1, 2],
                rotation_matrix[1, 1]
            )

            pitch = math.atan2(
                -rotation_matrix[2, 0],
                sy
            )

            yaw = 0.0

        return roll, pitch, yaw

    def publish_image(self, frame):
        success, encoded_image = cv2.imencode(
            '.jpg',
            frame,
            [
                int(cv2.IMWRITE_JPEG_QUALITY),
                85
            ]
        )

        if not success:
            self.get_logger().warn(
                'Image encoding failed'
            )
            return

        msg = CompressedImage()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        self.image_pub.publish(msg)

    def draw_center_lines(self, frame):
        cv2.line(
            frame,
            (self.center_x, 0),
            (self.center_x, self.frame_height),
            (255, 0, 0),
            2
        )

        cv2.line(
            frame,
            (0, self.center_y),
            (self.frame_width, self.center_y),
            (255, 0, 0),
            2
        )

    def calculate_marker_center(self, corner):
        points = corner[0]

        cx = int(
            (
                points[0][0]
                + points[1][0]
                + points[2][0]
                + points[3][0]
            ) / 4.0
        )

        cy = int(
            (
                points[0][1]
                + points[1][1]
                + points[2][1]
                + points[3][1]
            ) / 4.0
        )

        return cx, cy

    def calculate_marker_area(self, corner):
        points = corner[0]

        x1, y1 = points[0]
        x2, y2 = points[1]
        x3, y3 = points[2]
        x4, y4 = points[3]

        area = 0.5 * abs(
            x1 * y2
            + x2 * y3
            + x3 * y4
            + x4 * y1
            - y1 * x2
            - y2 * x3
            - y3 * x4
            - y4 * x1
        )

        return area

    def calculate_marker_width(self, corner):
        points = corner[0]

        p0 = points[0]
        p1 = points[1]
        p2 = points[2]
        p3 = points[3]

        top_width = math.sqrt(
            (p1[0] - p0[0]) ** 2
            + (p1[1] - p0[1]) ** 2
        )

        bottom_width = math.sqrt(
            (p2[0] - p3[0]) ** 2
            + (p2[1] - p3[1]) ** 2
        )

        left_height = math.sqrt(
            (p3[0] - p0[0]) ** 2
            + (p3[1] - p0[1]) ** 2
        )

        right_height = math.sqrt(
            (p2[0] - p1[0]) ** 2
            + (p2[1] - p1[1]) ** 2
        )

        marker_size_px = (
            top_width
            + bottom_width
            + left_height
            + right_height
        ) / 4.0

        return marker_size_px

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ArucoNode()

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
