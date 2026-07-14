#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


class Tb3Marker159F2PoseNode(Node):
    def __init__(self):
        super().__init__("tb3_marker159_f2_pose_node")

        self.declare_parameter("target_marker_id", 159)
        self.declare_parameter("tb3_marker_x", -0.16)
        self.declare_parameter("tb3_marker_y", 0.0)
        self.declare_parameter("f2_camera_x", 0.15)
        self.declare_parameter("f2_camera_y", 0.0)
        self.declare_parameter("target_timeout_sec", 0.70)
        self.declare_parameter("publish_period_sec", 0.10)

        self.target_marker_id = int(self.get_parameter("target_marker_id").value)
        self.tb3_marker_x = float(self.get_parameter("tb3_marker_x").value)
        self.tb3_marker_y = float(self.get_parameter("tb3_marker_y").value)
        self.f2_camera_x = float(self.get_parameter("f2_camera_x").value)
        self.f2_camera_y = float(self.get_parameter("f2_camera_y").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        self.tb3_pose_ok = False
        self.tb3_x = 0.0
        self.tb3_y = 0.0
        self.tb3_yaw = 0.0
        self.last_tb3_time = 0.0

        self.target_seen = False
        self.target_distance = 0.0
        self.target_bearing = 0.0
        self.last_target_time = 0.0
        self.last_target_raw = ""

        self.last_wall_pose = ""
        self.last_wall_time = 0.0

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10,
        )

        self.aruco_sub = self.create_subscription(
            String,
            "/f2/aruco_marker",
            self.aruco_callback,
            10,
        )

        self.wall_pose_sub = self.create_subscription(
            String,
            "/f2/relative_pose_wall",
            self.wall_pose_callback,
            10,
        )

        self.relative_pose_pub = self.create_publisher(
            String,
            "/f2/relative_pose",
            10,
        )

        self.localization_status_pub = self.create_publisher(
            String,
            "/f2/localization_status",
            10,
        )

        self.timer = self.create_timer(self.publish_period_sec, self.timer_callback)

        self.get_logger().info(
            "tb3_marker159_f2_pose_node started. "
            f"target_marker_id={self.target_marker_id}, "
            f"tb3_marker=({self.tb3_marker_x:.3f},{self.tb3_marker_y:.3f}), "
            f"f2_camera=({self.f2_camera_x:.3f},{self.f2_camera_y:.3f}), "
            f"target_timeout={self.target_timeout_sec:.2f}s"
        )

    def now_sec(self):
        return time.time()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def parse_key_values(self, text):
        result = {}

        cleaned = text.replace(";", ",").replace(" ", ",")
        parts = cleaned.split(",")

        for part in parts:
            part = part.strip()
            if "=" not in part:
                continue

            key, value = part.split("=", 1)
            key = key.strip().lower()
            value = value.strip()

            result[key] = value

        return result

    def get_float(self, data, keys, default=None):
        for key in keys:
            if key in data:
                try:
                    return float(data[key])
                except ValueError:
                    pass
        return default

    def get_int(self, data, keys, default=None):
        for key in keys:
            if key in data:
                try:
                    return int(float(data[key]))
                except ValueError:
                    pass
        return default

    def amcl_callback(self, msg):
        self.tb3_x = float(msg.pose.pose.position.x)
        self.tb3_y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        self.tb3_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.tb3_pose_ok = True
        self.last_tb3_time = self.now_sec()

    def aruco_callback(self, msg):
        text = msg.data.strip()
        data = self.parse_key_values(text)

        marker_id = self.get_int(data, ["id", "marker_id", "target_id"], None)
        if marker_id != self.target_marker_id:
            return

        distance = self.get_float(
            data,
            ["distance", "dist", "range", "range_m", "target_distance"],
            None,
        )

        bearing_deg = self.get_float(
            data,
            ["bearing", "bearing_deg", "bearing_yaw_deg", "angle_deg", "target_bearing"],
            None,
        )

        bearing_rad = self.get_float(
            data,
            ["bearing_rad", "angle_rad", "yaw_rad"],
            None,
        )

        rel_x = self.get_float(data, ["rel_x", "x", "tvec_z"], None)
        rel_y = self.get_float(data, ["rel_y", "y", "tvec_x"], None)

        if distance is None and rel_x is not None and rel_y is not None:
            distance = math.sqrt(rel_x * rel_x + rel_y * rel_y)

        if bearing_rad is None:
            if bearing_deg is not None:
                bearing_rad = math.radians(bearing_deg)
            elif rel_x is not None and rel_y is not None:
                bearing_rad = math.atan2(rel_y, rel_x)
            else:
                bearing_rad = 0.0

        if distance is None:
            return

        self.target_seen = True
        self.target_distance = float(distance)
        self.target_bearing = float(bearing_rad)
        self.last_target_time = self.now_sec()
        self.last_target_raw = text

    def wall_pose_callback(self, msg):
        self.last_wall_pose = msg.data.strip()
        self.last_wall_time = self.now_sec()

    def rotate_point(self, x, y, yaw):
        c = math.cos(yaw)
        s = math.sin(yaw)

        rx = c * x - s * y
        ry = s * x + c * y

        return rx, ry

    def compute_f2_pose_from_tb3_marker(self):
        if not self.tb3_pose_ok:
            return None

        now = self.now_sec()

        if now - self.last_target_time > self.target_timeout_sec:
            return None

        # 현재는 F2가 TB3 뒤 마커를 정면으로 따라간다는 가정.
        # 따라서 F2 yaw는 TB3 yaw와 거의 같다고 본다.
        f2_yaw = self.tb3_yaw

        # TB3 뒤쪽 159번 마커의 map 좌표
        marker_dx, marker_dy = self.rotate_point(
            self.tb3_marker_x,
            self.tb3_marker_y,
            self.tb3_yaw,
        )

        marker_map_x = self.tb3_x + marker_dx
        marker_map_y = self.tb3_y + marker_dy

        # F2 카메라 기준에서 마커까지의 벡터
        cam_to_marker_x = self.target_distance * math.cos(self.target_bearing)
        cam_to_marker_y = self.target_distance * math.sin(self.target_bearing)

        cam_to_marker_map_x, cam_to_marker_map_y = self.rotate_point(
            cam_to_marker_x,
            cam_to_marker_y,
            f2_yaw,
        )

        # F2 카메라 map 좌표 = 마커 map 좌표 - 카메라에서 마커까지의 map 벡터
        f2_camera_map_x = marker_map_x - cam_to_marker_map_x
        f2_camera_map_y = marker_map_y - cam_to_marker_map_y

        # F2 base 중심 = 카메라 위치 - base에서 camera까지의 offset
        f2_cam_offset_x, f2_cam_offset_y = self.rotate_point(
            self.f2_camera_x,
            self.f2_camera_y,
            f2_yaw,
        )

        f2_base_x = f2_camera_map_x - f2_cam_offset_x
        f2_base_y = f2_camera_map_y - f2_cam_offset_y

        return f2_base_x, f2_base_y, self.normalize_angle(f2_yaw)

    def publish_text(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def publish_fallback_wall_pose(self):
        if not self.last_wall_pose:
            self.publish_text(
                self.localization_status_pub,
                "LOCALIZATION,robot=f2,source=none,marker_seen=0,aruco_accepted=0,reason=no_tb3_marker_and_no_wall_pose",
            )
            return

        # 벽 마커 기반 pose를 최종 /f2/relative_pose로 그대로 전달
        self.publish_text(self.relative_pose_pub, self.last_wall_pose)

        self.publish_text(
            self.localization_status_pub,
            "LOCALIZATION,robot=f2,source=relative_pose_wall,marker_seen=0,aruco_accepted=0,reason=tb3_marker159_not_available",
        )

    def timer_callback(self):
        pose = self.compute_f2_pose_from_tb3_marker()

        if pose is None:
            self.publish_fallback_wall_pose()
            return

        x, y, yaw = pose

        age = self.now_sec() - self.last_target_time
        bearing_deg = math.degrees(self.target_bearing)

        text = (
            f"RELPOSE,"
            f"x={x:.3f},"
            f"y={y:.3f},"
            f"yaw={yaw:.3f},"
            f"yaw_deg={math.degrees(yaw):.2f},"
            f"source=tb3_marker159,"
            f"marker_id={self.target_marker_id},"
            f"marker_seen=1,"
            f"aruco_accepted=1,"
            f"distance={self.target_distance:.3f},"
            f"bearing_deg={bearing_deg:.2f},"
            f"age={age:.2f}"
        )

        self.publish_text(self.relative_pose_pub, text)

        status = (
            f"LOCALIZATION,"
            f"robot=f2,"
            f"source=tb3_marker159,"
            f"marker_seen=1,"
            f"aruco_accepted=1,"
            f"target_marker_id={self.target_marker_id},"
            f"distance={self.target_distance:.3f},"
            f"bearing_deg={bearing_deg:.2f},"
            f"tb3_x={self.tb3_x:.3f},"
            f"tb3_y={self.tb3_y:.3f},"
            f"tb3_yaw={self.tb3_yaw:.3f},"
            f"f2_x={x:.3f},"
            f"f2_y={y:.3f},"
            f"f2_yaw={yaw:.3f}"
        )

        self.publish_text(self.localization_status_pub, status)


def main():
    rclpy.init()
    node = Tb3Marker159F2PoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
