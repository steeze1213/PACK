#!/usr/bin/env python3
"""TB3 marker 159 based F1 pose fusion node.

Purpose:
  - Existing relative_pose_node publishes wall/static-marker based F1 pose to:
      /f1/relative_pose_wall

  - This node publishes the final F1 pose to:
      /f1/relative_pose

  - If F1 sees TB3 rear marker 159:
      TB3 /amcl_pose + TB3 marker offset + F1 camera relative target info
      -> F1 map pose

  - If marker 159 is not fresh:
      fallback to /f1/relative_pose_wall

Notes:
  - This node does not move the robot.
  - This node only publishes F1 pose string.
  - Dashboard, waypoint_drive_node, mission_manager keep reading /f1/relative_pose.
"""

import math
import re
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_xy(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def get_number_after_key(text, key):
    pattern = r'(?:^|[, ]+)' + re.escape(key) + r'\s*=\s*([-+]?\d+(?:\.\d+)?)'
    match = re.search(pattern, text)
    if not match:
        return None

    try:
        return float(match.group(1))
    except ValueError:
        return None


def get_int_after_key(text, key):
    value = get_number_after_key(text, key)
    if value is None:
        return None
    return int(value)


class TB3Marker159F1PoseNode(Node):
    def __init__(self):
        super().__init__("tb3_marker159_f1_pose_node")

        self.target_marker_id = int(
            self.declare_parameter("target_marker_id", 159).value
        )

        # TB3 base_link 기준 159번 마커 위치.
        # x < 0 이면 TB3 뒤쪽.
        self.tb3_marker_x = float(
            self.declare_parameter("tb3_marker_x", -0.16).value
        )
        self.tb3_marker_y = float(
            self.declare_parameter("tb3_marker_y", 0.0).value
        )

        # F1 base 기준 카메라 위치.
        self.f1_camera_x = float(
            self.declare_parameter("f1_camera_x", 0.15).value
        )
        self.f1_camera_y = float(
            self.declare_parameter("f1_camera_y", 0.0).value
        )

        self.target_timeout_sec = float(
            self.declare_parameter("target_timeout_sec", 0.70).value
        )
        self.wall_timeout_sec = float(
            self.declare_parameter("wall_timeout_sec", 1.00).value
        )
        self.publish_period_sec = float(
            self.declare_parameter("publish_period_sec", 0.10).value
        )

        # bearing 부호가 반대로 보이면 launch parameter에서 -1.0으로 바꾸면 됨.
        self.bearing_sign = float(
            self.declare_parameter("bearing_sign", 1.0).value
        )

        self.last_tb3_pose = None
        self.last_tb3_time = 0.0

        self.last_target = None
        self.last_target_time = 0.0

        self.last_wall_pose_msg = None
        self.last_wall_time = 0.0

        self.pose_pub = self.create_publisher(
            String,
            "/f1/relative_pose",
            20,
        )

        self.status_pub = self.create_publisher(
            String,
            "/f1/localization_status",
            10,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            20,
        )

        self.create_subscription(
            String,
            "/f1/aruco_marker",
            self.aruco_marker_callback,
            20,
        )

        self.create_subscription(
            String,
            "/f1/relative_pose_wall",
            self.wall_pose_callback,
            20,
        )

        self.create_timer(self.publish_period_sec, self.timer_callback)

        self.get_logger().info(
            "tb3_marker159_f1_pose_node started. "
            f"target_marker_id={self.target_marker_id}, "
            f"tb3_marker=({self.tb3_marker_x:.3f},{self.tb3_marker_y:.3f}), "
            f"f1_camera=({self.f1_camera_x:.3f},{self.f1_camera_y:.3f}), "
            f"target_timeout={self.target_timeout_sec:.2f}s"
        )

    def now_sec(self):
        return time.monotonic()

    def amcl_callback(self, msg):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)

        self.last_tb3_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": float(yaw),
        }
        self.last_tb3_time = self.now_sec()

    def aruco_marker_callback(self, msg):
        data = msg.data.strip()
        lower = data.lower()

        marker_id = None

        for key in ("marker_id", "id"):
            marker_id = get_int_after_key(lower, key)
            if marker_id is not None:
                break

        if marker_id != self.target_marker_id:
            return

        parsed = self.parse_target_marker_text(lower)

        if parsed is None:
            self.get_logger().warn(
                "Target marker 159 was seen but relative distance/bearing "
                f"could not be parsed. raw={data}"
            )
            return

        self.last_target = parsed
        self.last_target_time = self.now_sec()

    def parse_target_marker_text(self, text):
        # 우선 distance + bearing 형식을 사용한다.
        # 예: id=159,distance=0.276,bearing=-1.8
        distance = None
        bearing_deg = None

        for key in ("distance", "dist", "range"):
            distance = get_number_after_key(text, key)
            if distance is not None:
                break

        for key in ("bearing", "bearing_deg", "yaw_deg", "angle"):
            bearing_deg = get_number_after_key(text, key)
            if bearing_deg is not None:
                break

        if distance is not None and bearing_deg is not None:
            bearing_rad = math.radians(bearing_deg * self.bearing_sign)

            rel_x = distance * math.cos(bearing_rad)
            rel_y = distance * math.sin(bearing_rad)

            return {
                "mode": "distance_bearing",
                "distance": float(distance),
                "bearing_deg": float(bearing_deg),
                "rel_x": float(rel_x),
                "rel_y": float(rel_y),
                "raw": text,
            }

        # tvec 기반 형식도 지원한다.
        # 일반적으로 OpenCV camera frame에서 tvec_z가 전방 거리, tvec_x가 좌우.
        # 여기서는 robot frame 기준으로 rel_x=전방, rel_y=좌우로 변환한다.
        tvec_x = get_number_after_key(text, "tvec_x")
        tvec_z = get_number_after_key(text, "tvec_z")

        if tvec_x is not None and tvec_z is not None:
            rel_x = float(tvec_z)
            rel_y = float(tvec_x)

            distance = math.hypot(rel_x, rel_y)
            bearing_deg = math.degrees(math.atan2(rel_y, rel_x))

            return {
                "mode": "tvec_xz",
                "distance": float(distance),
                "bearing_deg": float(bearing_deg),
                "rel_x": float(rel_x),
                "rel_y": float(rel_y),
                "raw": text,
            }

        # rel_x, rel_y 또는 rel_x, rel_z 기반 형식도 지원한다.
        rel_x_value = get_number_after_key(text, "rel_x")
        rel_y_value = get_number_after_key(text, "rel_y")
        rel_z_value = get_number_after_key(text, "rel_z")

        if rel_x_value is not None and rel_y_value is not None:
            rel_x = float(rel_x_value)
            rel_y = float(rel_y_value)
            distance = math.hypot(rel_x, rel_y)
            bearing_deg = math.degrees(math.atan2(rel_y, rel_x))

            return {
                "mode": "rel_xy",
                "distance": float(distance),
                "bearing_deg": float(bearing_deg),
                "rel_x": float(rel_x),
                "rel_y": float(rel_y),
                "raw": text,
            }

        if rel_x_value is not None and rel_z_value is not None:
            # rel_z가 전방이고 rel_x가 좌우인 형식일 때를 대비.
            rel_x = float(rel_z_value)
            rel_y = float(rel_x_value)
            distance = math.hypot(rel_x, rel_y)
            bearing_deg = math.degrees(math.atan2(rel_y, rel_x))

            return {
                "mode": "rel_xz",
                "distance": float(distance),
                "bearing_deg": float(bearing_deg),
                "rel_x": float(rel_x),
                "rel_y": float(rel_y),
                "raw": text,
            }

        return None

    def wall_pose_callback(self, msg):
        self.last_wall_pose_msg = msg.data.strip()
        self.last_wall_time = self.now_sec()

    def target_is_fresh(self, now):
        if self.last_target is None:
            return False

        if now - self.last_target_time > self.target_timeout_sec:
            return False

        if self.last_tb3_pose is None:
            return False

        if now - self.last_tb3_time > 1.0:
            return False

        return True

    def wall_pose_is_fresh(self, now):
        if self.last_wall_pose_msg is None:
            return False

        if now - self.last_wall_time > self.wall_timeout_sec:
            return False

        return True

    def compute_f1_pose_from_tb3_marker(self):
        tb3_x = self.last_tb3_pose["x"]
        tb3_y = self.last_tb3_pose["y"]
        tb3_yaw = self.last_tb3_pose["yaw"]

        target = self.last_target

        # 1. TB3 base 기준 159번 마커 위치를 map 좌표로 변환
        marker_dx_map, marker_dy_map = rotate_xy(
            self.tb3_marker_x,
            self.tb3_marker_y,
            tb3_yaw,
        )

        marker_map_x = tb3_x + marker_dx_map
        marker_map_y = tb3_y + marker_dy_map

        # 2. F1은 TB3와 같은 방향을 보고 있다고 가정
        # 추종 상황에서는 F1이 TB3 뒤를 같은 방향으로 따라가기 때문에 우선 이 가정이 맞다.
        f1_yaw = tb3_yaw

        # 3. F1 카메라 기준에서 본 159 상대 위치를 map 좌표로 변환
        rel_map_x, rel_map_y = rotate_xy(
            target["rel_x"],
            target["rel_y"],
            f1_yaw,
        )

        # marker_map = camera_map + rel_map
        camera_map_x = marker_map_x - rel_map_x
        camera_map_y = marker_map_y - rel_map_y

        # 4. F1 base 기준 camera offset 보정
        cam_offset_map_x, cam_offset_map_y = rotate_xy(
            self.f1_camera_x,
            self.f1_camera_y,
            f1_yaw,
        )

        f1_map_x = camera_map_x - cam_offset_map_x
        f1_map_y = camera_map_y - cam_offset_map_y

        return {
            "x": f1_map_x,
            "y": f1_map_y,
            "yaw": f1_yaw,
            "marker_map_x": marker_map_x,
            "marker_map_y": marker_map_y,
            "distance": target["distance"],
            "bearing_deg": target["bearing_deg"],
            "rel_x": target["rel_x"],
            "rel_y": target["rel_y"],
            "mode": target["mode"],
        }

    def publish_target_based_pose(self):
        pose = self.compute_f1_pose_from_tb3_marker()

        yaw_deg = math.degrees(pose["yaw"])

        msg = String()
        msg.data = (
            "RELPOSE,"
            f"x={pose['x']:.3f},"
            f"y={pose['y']:.3f},"
            f"yaw={pose['yaw']:.4f},"
            f"yaw_deg={yaw_deg:.2f},"
            "source=tb3_marker159,"
            f"marker_id={self.target_marker_id},"
            "used_ids=159,"
            "marker_count_used=1,"
            "pnp_label=tb3_marker159,"
            "active_face=tb3,"
            "marker_seen=1,"
            "aruco_accepted=1,"
            f"rel_x={pose['rel_x']:.3f},"
            f"rel_y={pose['rel_y']:.3f},"
            "rel_z=0.000,"
            f"bearing_yaw_deg={pose['bearing_deg']:.2f},"
            "marker_roll=0.0000,"
            "marker_roll_deg=0.00,"
            "marker_pitch=0.0000,"
            "marker_pitch_deg=0.00,"
            f"marker_yaw={pose['yaw']:.4f},"
            f"marker_yaw_deg={yaw_deg:.2f},"
            "rvec_x=0.000000,"
            "rvec_y=0.000000,"
            "rvec_z=0.000000,"
            "tvec_x=0.000000,"
            "tvec_y=0.000000,"
            "tvec_z=0.000000,"
            f"marker_x={pose['marker_map_x']:.3f},"
            f"marker_y={pose['marker_map_y']:.3f},"
            f"marker_map_yaw={pose['yaw']:.4f},"
            f"marker_map_yaw_deg={yaw_deg:.2f},"
            "marker_local_x=0.000,"
            "marker_local_y=0.000,"
            "marker_local_z=0.000,"
            f"camera_x={pose['x'] + self.f1_camera_x:.3f},"
            f"camera_y={pose['y'] + self.f1_camera_y:.3f},"
            f"robot_x_raw={pose['x']:.3f},"
            f"robot_y_raw={pose['y']:.3f},"
            f"robot_yaw_raw={pose['yaw']:.4f},"
            f"robot_yaw_raw_deg={yaw_deg:.2f},"
            f"robust_x={pose['x']:.3f},"
            f"robust_y={pose['y']:.3f},"
            f"robust_yaw={pose['yaw']:.4f},"
            f"robust_yaw_deg={yaw_deg:.2f},"
            "raw_buffer_count=1,"
            "control_jump_count=0,"
            "yaw_error_from_rvec=0.0000,"
            "yaw_error_from_rvec_deg=0.00,"
            "left_count=0,"
            "right_count=0,"
            "left_delta=0,"
            "right_delta=0,"
            "left_dist=0.0000,"
            "right_dist=0.0000,"
            "center_dist=0.0000,"
            "delta_yaw=0.0000,"
            "delta_yaw_deg=0.00,"
            "reproj_error=0.00,"
            "cmd=tb3_marker159"
        )

        self.pose_pub.publish(msg)

        status = String()
        status.data = (
            "LOCALIZATION_STATUS,"
            "robot=f1,"
            "ok=1,"
            "source=tb3_marker159,"
            f"target_id={self.target_marker_id},"
            f"distance={pose['distance']:.3f},"
            f"bearing_deg={pose['bearing_deg']:.2f},"
            f"x={pose['x']:.3f},"
            f"y={pose['y']:.3f},"
            f"yaw={pose['yaw']:.4f}"
        )
        self.status_pub.publish(status)

    def publish_wall_fallback_pose(self):
        msg = String()
        msg.data = self.last_wall_pose_msg
        self.pose_pub.publish(msg)

        status = String()
        status.data = (
            "LOCALIZATION_STATUS,"
            "robot=f1,"
            "ok=1,"
            "source=wall_fallback"
        )
        self.status_pub.publish(status)

    def publish_lost_status(self):
        status = String()
        status.data = (
            "LOCALIZATION_STATUS,"
            "robot=f1,"
            "ok=0,"
            "source=lost,"
            "reason=no_fresh_tb3_marker159_or_wall_pose"
        )
        self.status_pub.publish(status)

    def timer_callback(self):
        now = self.now_sec()

        if self.target_is_fresh(now):
            self.publish_target_based_pose()
            return

        if self.wall_pose_is_fresh(now):
            self.publish_wall_fallback_pose()
            return

        self.publish_lost_status()


def main(args=None):
    rclpy.init(args=args)

    node = TB3Marker159F1PoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
