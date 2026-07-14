#!/usr/bin/env python3
# flake8: noqa: E501
"""F1 target follow with one-shot wall-pose recovery and map return-home."""

import math
import os
import time
from collections import deque
from statistics import median

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class F1HybridFollowPoseNode(Node):
    """Follow marker 159, recover a stable pose only when lost, then reacquire."""

    STATES = (
        'PAUSED',
        'FOLLOW_TARGET',
        'TARGET_LOST',
        'POSE_RECOVERY',
        'REACQUIRE_TARGET',
        'LOAD_WAIT',
        'RETURN_HOME',
        'DONE',
    )
    VALID_COMMANDS = ('w', 'q', 'e', 's')
    TRUE_VALUES = ('1', 'true', 'yes', 'on', 'start', 'done', 'complete', 'completed')

    def __init__(self):
        super().__init__('f1_hybrid_follow_pose_node')
        self._declare_parameters()
        self._load_parameters()

        self.cmd_pub = self.create_publisher(String, '/robot_cmd', 20)
        self.status_pub = self.create_publisher(String, '/hybrid_status', 10)
        self.create_subscription(String, '/target_marker', self.target_callback, 20)
        self.create_subscription(String, '/aruco_marker', self.aruco_callback, 10)
        self.create_subscription(String, '/aruco_multi_markers', self.multi_callback, 20)
        self.create_subscription(String, '/relative_pose', self.pose_callback, 20)
        self.create_subscription(String, '/encoder_counts', self.encoder_callback, 30)
        self.create_subscription(String, '/load_done', self.load_done_callback, 10)
        self.create_subscription(String, '/mission/load_done', self.load_done_callback, 10)
        self.create_subscription(String, '/return_home', self.return_home_callback, 10)
        self.create_subscription(String, '/mission/return_home', self.return_home_callback, 10)
        self.create_subscription(String, '/load_wait', self.load_wait_callback, 10)
        self.create_subscription(String, '/mission/load_wait', self.load_wait_callback, 10)
        self.create_subscription(String, '/reanchor', self.reanchor_callback, 10)
        self.create_subscription(String, '/mission_enable', self.mission_enable_callback, 10)

        now = time.monotonic()
        self.state = 'FOLLOW_TARGET'
        self.mission_enabled = bool(
            self.get_parameter('start_enabled').value
        )
        if not self.mission_enabled:
            self.state = 'PAUSED'
        self.state_enter_time = now
        self.target_seen = False
        self.target_distance = 0.0
        self.target_bearing_deg = 0.0
        self.last_target_seen_time = now
        self.last_target_msg_time = 0.0
        self.target_hold_since = None

        self.latest_multi_time = 0.0
        self.latest_marker_ids = []
        self.latest_face = None
        self.latest_group = ()
        self.last_pose_input = None
        self.last_pose_input_time = 0.0
        self.last_aruco_seq = -1
        self.previous_left_count = None
        self.previous_right_count = None

        # Five slots are required when the explicitly enabled 2-marker initial
        # recovery fallback is used; normal 3+ marker recovery uses 3-5 samples.
        self.recovery_samples = deque(maxlen=5)
        self.recovery_face = None
        self.recovery_group = ()
        self.recovery_pose = None
        self.recovery_locked = False
        self.recovery_lock_reason = ''
        self.allow_face_reanchor_once = False
        self.recovery_two_marker_session = False

        self.control_pose = None
        self.active_goal = None
        self.return_home_started = False
        self.return_home_recovery_pending = False
        self.last_command = None
        self.last_command_log_time = 0.0
        self.last_status_time = 0.0
        self.last_reject_log = {}

        self.create_timer(self.control_period_sec, self.control_loop)
        self.create_timer(0.5, self.publish_status)
        self.publish_command('s', 'startup')
        self.get_logger().info(
            'F1 hybrid node started: target=159, direct command set=w/q/e/s, '
            'wall pose is ignored for FOLLOW command decisions'
        )

    def _declare_parameters(self):
        values = {
            'target_marker_id': 159,
            'lost_timeout_sec': 0.70,
            'target_fresh_timeout_sec': 0.35,
            'follow_stop_distance_m': 0.09,
            'follow_forward_distance_m': 0.12,
            'follow_bearing_tolerance_deg': 7.0,
            'follow_hard_bearing_deg': 35.0,
            'control_period_sec': 0.05,
            'recovery_sample_count': 4,
            'recovery_min_markers': 3,
            'recovery_max_reproj_px': 2.0,
            'recovery_pos_spread_m': 0.05,
            'recovery_yaw_spread_deg': 5.0,
            'multi_pose_sync_sec': 0.35,
            'marker_group_adjacent_gap': 2,
            'allow_two_marker_initial': False,
            'marker_reference_path': '',
            'reacquire_forward_m': 0.25,
            'reacquire_yaw_offset_deg': 0.0,
            'reacquire_timeout_sec': 8.0,
            'goal_position_tolerance_m': 0.07,
            'goal_yaw_tolerance_deg': 8.0,
            'goal_bearing_tolerance_deg': 10.0,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_yaw': 0.0,
            'home_position_tolerance_m': 0.08,
            'home_yaw_tolerance_deg': 8.0,
            'odom_max_step_m': 0.12,
            'odom_max_yaw_step_deg': 20.0,
            'wheel_circumference_m': 0.21,
            'encoder_counts_per_turn': 3600.0,
            'wheel_base_m': 0.23,
            'left_encoder_scale': 1.0,
            'right_encoder_scale': 0.423,
            'max_encoder_delta_per_msg': 1000,
            'load_wait_after_target_hold_sec': 0.0,
            'load_hold_min_distance_m': 0.06,
            'load_hold_max_distance_m': 0.10,
            # Standalone test launch behavior remains enabled by default.
            'start_enabled': True,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        def get(name):
            return self.get_parameter(name).value

        self.target_marker_id = int(get('target_marker_id'))
        self.lost_timeout_sec = float(get('lost_timeout_sec'))
        self.target_fresh_timeout_sec = float(get('target_fresh_timeout_sec'))
        self.follow_stop_distance = float(get('follow_stop_distance_m'))
        self.follow_forward_distance = float(get('follow_forward_distance_m'))
        self.follow_bearing_tolerance = float(get('follow_bearing_tolerance_deg'))
        self.follow_hard_bearing = float(get('follow_hard_bearing_deg'))
        self.control_period_sec = float(get('control_period_sec'))
        self.recovery_sample_count = max(3, min(5, int(get('recovery_sample_count'))))
        self.recovery_min_markers = max(3, int(get('recovery_min_markers')))
        self.recovery_max_reproj = float(get('recovery_max_reproj_px'))
        self.recovery_pos_spread = float(get('recovery_pos_spread_m'))
        self.recovery_yaw_spread = float(get('recovery_yaw_spread_deg'))
        self.multi_pose_sync_sec = float(get('multi_pose_sync_sec'))
        self.marker_group_adjacent_gap = int(get('marker_group_adjacent_gap'))
        self.allow_two_marker_initial = bool(get('allow_two_marker_initial'))
        self.reacquire_forward = float(get('reacquire_forward_m'))
        self.reacquire_yaw_offset = math.radians(float(get('reacquire_yaw_offset_deg')))
        self.reacquire_timeout = float(get('reacquire_timeout_sec'))
        self.goal_position_tolerance = float(get('goal_position_tolerance_m'))
        self.goal_yaw_tolerance = float(get('goal_yaw_tolerance_deg'))
        self.goal_bearing_tolerance = float(get('goal_bearing_tolerance_deg'))
        self.home_pose = (
            float(get('home_x')), float(get('home_y')), float(get('home_yaw'))
        )
        self.home_position_tolerance = float(get('home_position_tolerance_m'))
        self.home_yaw_tolerance = float(get('home_yaw_tolerance_deg'))
        self.odom_max_step = float(get('odom_max_step_m'))
        self.odom_max_yaw_step = float(get('odom_max_yaw_step_deg'))
        self.wheel_circumference = float(get('wheel_circumference_m'))
        self.encoder_counts_per_turn = float(get('encoder_counts_per_turn'))
        self.wheel_base = float(get('wheel_base_m'))
        self.left_encoder_scale = float(get('left_encoder_scale'))
        self.right_encoder_scale = float(get('right_encoder_scale'))
        self.max_encoder_delta = int(get('max_encoder_delta_per_msg'))
        self.load_wait_after_hold = float(get('load_wait_after_target_hold_sec'))
        self.load_hold_min = float(get('load_hold_min_distance_m'))
        self.load_hold_max = float(get('load_hold_max_distance_m'))
        path = str(get('marker_reference_path')).strip()
        self.marker_faces = self._load_marker_faces(path)

    def _load_marker_faces(self, path):
        if not path:
            path = os.path.expanduser(
                '~/robot_ws/src/encoder_bridge/config/aruco_reference.yaml'
            )
        faces = {}
        try:
            with open(os.path.expanduser(path), 'r') as stream:
                data = yaml.safe_load(stream) or {}
            for marker_id, pose in data.get('aruco_marker_pose', {}).items():
                yaw = float(pose.get('yaw', 0.0))
                faces[int(marker_id)] = int(round(self.normalize(yaw) / (math.pi / 2.0))) % 4
            self.get_logger().info(f'Loaded {len(faces)} marker face references from {path}')
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().error(f'Marker reference load failed: {error}')
        return faces

    @staticmethod
    def parse(data, prefix):
        if not data.startswith(prefix + ','):
            return None
        result = {}
        for item in data.split(',')[1:]:
            if '=' in item:
                key, value = item.split('=', 1)
                result[key.strip()] = value.strip()
        return result

    @staticmethod
    def normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @classmethod
    def signal_true(cls, data):
        value = data.strip().lower()
        if value in cls.TRUE_VALUES:
            return True
        parsed = cls.parse(data.strip(), 'MISSION')
        return parsed is not None and parsed.get('value', '').lower() in cls.TRUE_VALUES

    def target_callback(self, msg):
        parsed = self.parse(msg.data.strip(), 'TARGET_MARKER')
        if parsed is None:
            return
        now = time.monotonic()
        self.last_target_msg_time = now
        try:
            seen = (
                int(parsed.get('detected', 0)) == 1
                and int(parsed.get('id', -1)) == self.target_marker_id
            )
            distance = float(parsed.get('planar_distance', 0.0))
            bearing = float(parsed.get('bearing_yaw_deg', 0.0))
            seen = seen and distance > 0.0
        except ValueError:
            seen = False
            distance = 0.0
            bearing = 0.0

        if seen:
            self.target_distance = distance
            self.target_bearing_deg = bearing
            self.last_target_seen_time = now
            if not self.target_seen:
                self.get_logger().info(
                    f'target marker seen: id={self.target_marker_id}, '
                    f'distance={distance:.3f}, bearing={bearing:.1f}'
                )
            self.target_seen = True
            if self.state in ('TARGET_LOST', 'POSE_RECOVERY', 'REACQUIRE_TARGET'):
                self.transition('FOLLOW_TARGET', 'target marker reacquired')
        elif self.target_seen:
            self.target_seen = False
            self.get_logger().warn(f'target marker lost: id={self.target_marker_id}')

    def aruco_callback(self, _msg):
        # Kept for topic compatibility and diagnostics. Single-marker pose is
        # deliberately never admitted to the recovery sample buffer.
        return

    def multi_callback(self, msg):
        parsed = self.parse(msg.data.strip(), 'MULTI_ARUCO')
        if parsed is None:
            return
        try:
            count = int(parsed.get('count', 0))
            detected = int(parsed.get('detected', 0)) == 1
            marker_ids = [int(parsed[f'm{i}_id']) for i in range(count)] if detected else []
        except (KeyError, ValueError):
            marker_ids = []
        self.latest_multi_time = time.monotonic()
        self.latest_marker_ids = marker_ids
        self.latest_face = self.dominant_face(marker_ids)
        self.latest_group = tuple(sorted(marker_ids))

    def dominant_face(self, marker_ids):
        counts = {}
        for marker_id in marker_ids:
            face = self.marker_faces.get(marker_id)
            if face is not None:
                counts[face] = counts.get(face, 0) + 1
        if not counts:
            return None
        return max(sorted(counts), key=lambda face: counts[face])

    def pose_callback(self, msg):
        parsed = self.parse(msg.data.strip(), 'RELPOSE')
        if parsed is None:
            return
        now = time.monotonic()
        try:
            pose = (
                float(parsed['x']), float(parsed['y']), float(parsed['yaw'])
            )
        except (KeyError, ValueError):
            return

        self.last_pose_input = pose
        self.last_pose_input_time = now

        collecting = self.state == 'POSE_RECOVERY' or (
            self.state == 'RETURN_HOME' and self.return_home_recovery_pending
        )
        if not collecting:
            return
        self.consider_recovery_candidate(parsed, now)

    def encoder_callback(self, msg):
        parts = msg.data.strip().split(',')
        if len(parts) != 6 or parts[0] != 'ENC':
            return
        try:
            left_count = int(parts[1])
            right_count = int(parts[2])
            command = parts[5].strip().lower()
        except ValueError:
            return
        if self.previous_left_count is None or self.previous_right_count is None:
            self.previous_left_count = left_count
            self.previous_right_count = right_count
            return
        left_delta = left_count - self.previous_left_count
        right_delta = right_count - self.previous_right_count
        self.previous_left_count = left_count
        self.previous_right_count = right_count
        if self.control_pose is None or self.state not in ('REACQUIRE_TARGET', 'RETURN_HOME'):
            return
        if command not in ('w', 'q', 'e'):
            return
        if abs(left_delta) > self.max_encoder_delta or abs(right_delta) > self.max_encoder_delta:
            self.log_reject(
                'encoder_count_jump',
                f'encoder count jump left={left_delta}, right={right_delta}',
            )
            return
        left_distance = (
            -left_delta * self.left_encoder_scale
            / self.encoder_counts_per_turn * self.wheel_circumference
        )
        right_distance = (
            right_delta * self.right_encoder_scale
            / self.encoder_counts_per_turn * self.wheel_circumference
        )
        center_distance = (left_distance + right_distance) / 2.0
        dyaw = (right_distance - left_distance) / self.wheel_base
        if (
            abs(center_distance) > self.odom_max_step
            or abs(math.degrees(dyaw)) > self.odom_max_yaw_step
        ):
            self.log_reject(
                'odom_jump',
                f'encoder odom jump {center_distance:.3f}m/{math.degrees(dyaw):.1f}deg',
            )
            return
        x, y, yaw = self.control_pose
        middle_yaw = yaw + dyaw / 2.0
        self.control_pose = (
            x + center_distance * math.cos(middle_yaw),
            y + center_distance * math.sin(middle_yaw),
            self.normalize(yaw + dyaw),
        )

    def consider_recovery_candidate(self, parsed, now):
        try:
            accepted = int(parsed.get('aruco_accepted', 0)) == 1
            sequence = int(parsed.get('aruco_seq', -1))
            reproj = float(parsed.get('reproj_error', 999.0))
            command = parsed.get('cmd', '')
            marker_count = int(command.split(':', 1)[1]) if command.startswith('multi:') else 0
        except (ValueError, IndexError):
            self.log_reject('parse', 'invalid relative pose quality fields')
            return
        if not accepted:
            self.log_reject('not_accepted', 'aruco_accepted=0')
            return
        if sequence == self.last_aruco_seq:
            return
        self.last_aruco_seq = sequence
        if now - self.latest_multi_time > self.multi_pose_sync_sec:
            self.log_reject('sync', 'multi-marker metadata is stale')
            return
        if marker_count < self.recovery_min_markers:
            two_marker_recovery = (
                marker_count == 2 and self.allow_two_marker_initial
                and not self.recovery_locked
            )
            if not two_marker_recovery:
                self.log_reject('count', f'markers={marker_count} < {self.recovery_min_markers}')
                return
            self.recovery_two_marker_session = True
        if marker_count == 1:
            self.log_reject('single', 'single-marker pose never updates position/yaw')
            return
        if reproj > self.recovery_max_reproj:
            self.log_reject('reproj', f'reproj={reproj:.2f}px > {self.recovery_max_reproj:.2f}px')
            return
        try:
            main_marker_id = int(parsed.get('marker_id', -1))
        except ValueError:
            main_marker_id = -1
        face = self.marker_faces.get(main_marker_id, self.latest_face)
        group = tuple(
            marker_id for marker_id in self.latest_group
            if self.marker_faces.get(marker_id) == face
        )
        if not group and main_marker_id >= 0:
            group = (main_marker_id,)
        if face is None:
            self.log_reject('face_unknown', f'face unknown for ids={list(group)}')
            return
        if self.recovery_face is not None and face != self.recovery_face:
            if not self.allow_face_reanchor_once:
                self.get_logger().warn(
                    f'face switch blocked: locked_candidate_face={self.recovery_face}, '
                    f'incoming_face={face}, ids={list(group)}'
                )
                return
            self.allow_face_reanchor_once = False
            self.clear_recovery_candidates('manual face reanchor')
        if self.recovery_group and not self.groups_compatible(self.recovery_group, group):
            self.log_reject(
                'group', f'non-adjacent marker group {list(group)} vs {list(self.recovery_group)}'
            )
            return
        try:
            candidate_pose = (
                float(parsed.get('robot_x_raw', parsed['x'])),
                float(parsed.get('robot_y_raw', parsed['y'])),
                float(parsed.get('robot_yaw_raw', parsed['yaw'])),
            )
        except (KeyError, ValueError):
            self.log_reject('pose_parse', 'candidate pose parse failed')
            return
        if self.recovery_face is None:
            self.recovery_face = face
            self.recovery_group = group
        self.recovery_samples.append(candidate_pose)
        self.recovery_group = group
        required_samples = 5 if self.recovery_two_marker_session else self.recovery_sample_count
        self.get_logger().info(
            f'recovery pose candidate: {len(self.recovery_samples)}/{required_samples}, '
            f'x={candidate_pose[0]:.3f}, y={candidate_pose[1]:.3f}, '
            f'yaw={math.degrees(candidate_pose[2]):.1f}deg, face={face}, '
            f'ids={list(group)}, markers={marker_count}, reproj={reproj:.2f}px'
        )
        self.try_lock_recovery_pose()

    def groups_compatible(self, first, second):
        if set(first).intersection(second):
            return True
        return min(abs(a - b) for a in first for b in second) <= self.marker_group_adjacent_gap

    def try_lock_recovery_pose(self):
        required_samples = 5 if self.recovery_two_marker_session else self.recovery_sample_count
        if self.recovery_locked or len(self.recovery_samples) < required_samples:
            return
        xs = [sample[0] for sample in self.recovery_samples]
        ys = [sample[1] for sample in self.recovery_samples]
        yaws = [sample[2] for sample in self.recovery_samples]
        center_x, center_y = median(xs), median(ys)
        center_yaw = math.atan2(
            sum(math.sin(yaw) for yaw in yaws), sum(math.cos(yaw) for yaw in yaws)
        )
        pos_spread = max(
            math.hypot(x - center_x, y - center_y) for x, y in zip(xs, ys)
        )
        yaw_spread = max(abs(math.degrees(self.normalize(yaw - center_yaw))) for yaw in yaws)
        if pos_spread > self.recovery_pos_spread:
            self.log_reject('spread_pos', f'pos spread={pos_spread:.3f}m')
            self.recovery_samples.popleft()
            return
        if yaw_spread > self.recovery_yaw_spread:
            self.log_reject('spread_yaw', f'yaw spread={yaw_spread:.1f}deg')
            self.recovery_samples.popleft()
            return
        self.recovery_pose = (center_x, center_y, center_yaw)
        self.control_pose = self.recovery_pose
        self.recovery_locked = True
        self.recovery_lock_reason = 'stable_multi_marker'
        self.get_logger().warn(
            f'recovery pose locked: x={center_x:.3f}, y={center_y:.3f}, '
            f'yaw={math.degrees(center_yaw):.1f}deg, face={self.recovery_face}, '
            f'pos_spread={pos_spread:.3f}m, yaw_spread={yaw_spread:.1f}deg; '
            'future camera candidates cannot overwrite this lock'
        )
        if self.state == 'POSE_RECOVERY':
            self.active_goal = self.make_reacquire_goal(self.recovery_pose)
            self.transition('REACQUIRE_TARGET', 'stable recovery pose locked')
        elif self.state == 'RETURN_HOME':
            self.return_home_recovery_pending = False
            self.active_goal = self.home_pose
            self.get_logger().info('return_home pose lock complete; direct home control enabled')

    def make_reacquire_goal(self, pose):
        x, y, yaw = pose
        return (
            x + self.reacquire_forward * math.cos(yaw),
            y + self.reacquire_forward * math.sin(yaw),
            self.normalize(yaw + self.reacquire_yaw_offset),
        )

    def clear_recovery_candidates(self, reason):
        self.recovery_samples.clear()
        self.recovery_face = None
        self.recovery_group = ()
        self.recovery_two_marker_session = False
        self.last_aruco_seq = -1
        self.get_logger().info(f'recovery candidates cleared: {reason}')

    def reset_recovery_lock(self, reason, manual=False):
        self.recovery_pose = None
        self.recovery_locked = False
        self.recovery_lock_reason = ''
        self.control_pose = None
        self.active_goal = None
        self.allow_face_reanchor_once = manual
        self.clear_recovery_candidates(reason)

    def load_wait_callback(self, msg):
        if self.signal_true(msg.data):
            self.transition('LOAD_WAIT', 'load wait requested')

    def load_done_callback(self, msg):
        if not self.signal_true(msg.data):
            return
        self.get_logger().warn(f'load_done received: {msg.data.strip()}')
        self.start_return_home('load_done received')

    def return_home_callback(self, msg):
        if self.signal_true(msg.data):
            self.start_return_home('return_home command received')

    def reanchor_callback(self, msg):
        if not self.signal_true(msg.data):
            return
        if self.state not in ('POSE_RECOVERY', 'RETURN_HOME'):
            self.get_logger().warn(f'manual reanchor ignored in state={self.state}')
            return
        self.reset_recovery_lock('manual reanchor command', manual=True)
        if self.state == 'RETURN_HOME':
            self.return_home_recovery_pending = True
        self.get_logger().warn('manual reanchor accepted; one face change is now allowed')

    def mission_enable_callback(self, msg):
        enabled = self.signal_true(msg.data)
        if enabled == self.mission_enabled:
            return
        self.mission_enabled = enabled
        if enabled:
            self.target_seen = False
            self.last_target_seen_time = time.monotonic()
            self.transition('FOLLOW_TARGET', 'mission enabled')
        else:
            self.transition('PAUSED', 'mission disabled')
            self.publish_command('s', 'mission disabled')

    def start_return_home(self, reason):
        if self.state == 'DONE':
            return
        self.return_home_started = True
        self.return_home_recovery_pending = True
        self.reset_recovery_lock('return_home fresh recovery', manual=True)
        self.get_logger().warn(f'return_home started: {reason}')
        self.transition('RETURN_HOME', reason)

    def transition(self, new_state, reason):
        if new_state not in self.STATES or new_state == self.state:
            return
        old_state = self.state
        self.state = new_state
        self.state_enter_time = time.monotonic()
        self.target_hold_since = None
        self.publish_command('s', f'state transition {old_state}->{new_state}')
        self.get_logger().warn(f'STATE changed: {old_state} -> {new_state}; reason={reason}')
        if new_state == 'POSE_RECOVERY':
            self.reset_recovery_lock('target lost recovery')
        if new_state == 'DONE':
            self.get_logger().warn('home arrived; STATE=DONE')

    def target_is_fresh(self, now):
        return self.target_seen and now - self.last_target_seen_time <= self.target_fresh_timeout_sec

    def control_loop(self):
        now = time.monotonic()
        if not self.mission_enabled or self.state == 'PAUSED':
            self.publish_command('s', 'mission paused')
        elif self.state == 'FOLLOW_TARGET':
            self.control_follow(now)
        elif self.state == 'TARGET_LOST':
            self.publish_command('s', 'target lost stop')
            self.transition('POSE_RECOVERY', 'robot stopped after target loss')
        elif self.state == 'POSE_RECOVERY':
            self.publish_command('s', 'collecting stable recovery pose')
        elif self.state == 'REACQUIRE_TARGET':
            self.control_reacquire(now)
        elif self.state == 'LOAD_WAIT':
            self.publish_command('s', 'waiting for load_done')
        elif self.state == 'RETURN_HOME':
            self.control_return_home()
        else:
            self.publish_command('s', 'DONE hold')

    def control_follow(self, now):
        if not self.target_is_fresh(now):
            self.publish_command('s', 'target stale')
            if now - self.last_target_seen_time >= self.lost_timeout_sec:
                self.get_logger().warn(
                    f'target marker lost timeout: {now - self.last_target_seen_time:.2f}s'
                )
                self.transition('TARGET_LOST', 'lost_timeout_sec exceeded')
            return
        bearing = self.target_bearing_deg
        distance = self.target_distance
        if abs(bearing) > self.follow_hard_bearing:
            self.publish_command('s', f'target bearing hard limit {bearing:.1f}deg')
        elif abs(bearing) > self.follow_bearing_tolerance:
            # Existing camera convention: positive bearing pivots right.
            self.publish_command('e' if bearing > 0.0 else 'q', 'target center alignment')
        elif distance > self.follow_forward_distance:
            self.publish_command('w', 'target follow forward')
        else:
            self.publish_command('s', f'target distance hold {distance:.3f}m')
        if self.load_wait_after_hold <= 0.0:
            return
        in_hold = self.load_hold_min <= distance <= self.load_hold_max
        if not in_hold:
            self.target_hold_since = None
        elif self.target_hold_since is None:
            self.target_hold_since = now
        elif now - self.target_hold_since >= self.load_wait_after_hold:
            self.transition('LOAD_WAIT', 'target held at configured load distance')

    def control_reacquire(self, now):
        if self.target_is_fresh(now):
            self.transition('FOLLOW_TARGET', 'target marker reacquired during motion')
            return
        if now - self.state_enter_time > self.reacquire_timeout:
            self.publish_command('s', 'reacquire timeout')
            return
        if not self.recovery_locked or self.control_pose is None or self.active_goal is None:
            self.publish_command('s', 'recovery lock unavailable')
            return
        command, arrived = self.goal_command(
            self.active_goal, self.goal_position_tolerance, self.goal_yaw_tolerance
        )
        self.publish_command(command, 'reacquire direct pose control')
        if arrived:
            self.publish_command('s', 'reacquire pose reached; waiting for target')

    def control_return_home(self):
        if self.return_home_recovery_pending:
            self.publish_command('s', 'return_home recovery pose collection')
            return
        if not self.recovery_locked or self.control_pose is None:
            self.publish_command('s', 'return_home pose unavailable')
            return
        command, arrived = self.goal_command(
            self.home_pose, self.home_position_tolerance, self.home_yaw_tolerance
        )
        self.publish_command(command, 'return_home direct pose control')
        if arrived:
            self.transition('DONE', 'home position and yaw reached')

    def goal_command(self, goal, position_tolerance, yaw_tolerance_deg):
        x, y, yaw = self.control_pose
        dx, dy = goal[0] - x, goal[1] - y
        distance = math.hypot(dx, dy)
        if distance > position_tolerance:
            bearing_error = math.degrees(self.normalize(math.atan2(dy, dx) - yaw))
            if abs(bearing_error) > self.goal_bearing_tolerance:
                return ('q' if bearing_error > 0.0 else 'e'), False
            return 'w', False
        yaw_error = math.degrees(self.normalize(goal[2] - yaw))
        if abs(yaw_error) > yaw_tolerance_deg:
            return ('q' if yaw_error > 0.0 else 'e'), False
        return 's', True

    def publish_command(self, command, reason):
        if command not in self.VALID_COMMANDS:
            command = 's'
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        now = time.monotonic()
        if command != self.last_command or now - self.last_command_log_time >= 2.0:
            self.get_logger().info(
                f'command output: cmd={command}, state={self.state}, reason={reason}'
            )
            self.last_command_log_time = now
        self.last_command = command

    def log_reject(self, key, reason):
        now = time.monotonic()
        if now - self.last_reject_log.get(key, 0.0) >= 1.0:
            self.get_logger().warn(f'recovery pose rejected reason: {reason}')
            self.last_reject_log[key] = now

    def publish_status(self):
        pose = self.control_pose or (float('nan'),) * 3
        goal = self.active_goal or (float('nan'),) * 3
        msg = String()
        msg.data = (
            f'HYBRID_STATUS,state={self.state},cmd={self.last_command or "s"},'
            f'target_seen={1 if self.target_seen else 0},'
            f'recovery_locked={1 if self.recovery_locked else 0},'
            f'recovery_face={self.recovery_face if self.recovery_face is not None else -1},'
            f'control_x={pose[0]:.3f},control_y={pose[1]:.3f},control_yaw={pose[2]:.4f},'
            f'goal_x={goal[0]:.3f},goal_y={goal[1]:.3f},goal_yaw={goal[2]:.4f}'
        )
        self.status_pub.publish(msg)

    def shutdown(self):
        for _ in range(3):
            self.publish_command('s', 'node shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = F1HybridFollowPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
