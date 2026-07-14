#!/usr/bin/env python3

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, String

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass(order=True)
class NodeAStar:
    f: float
    h: float
    position: GridPoint = field(compare=False)
    g: float = field(default=0.0, compare=False)
    parent: Optional["NodeAStar"] = field(default=None, compare=False)


def run_astar(
    map_data: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    width: int,
    height: int,
    cost_map: Optional[np.ndarray] = None,
) -> Optional[List[GridPoint]]:
    if not (0 <= start[0] < height and 0 <= start[1] < width):
        return None

    if not (0 <= goal[0] < height and 0 <= goal[1] < width):
        return None

    if map_data[start[0], start[1]] != 0:
        return None

    if map_data[goal[0], goal[1]] != 0:
        return None

    start_h = math.hypot(start[0] - goal[0], start[1] - goal[1])
    start_node = NodeAStar(
        f=start_h,
        h=start_h,
        position=start,
        g=0.0,
        parent=None,
    )

    open_list: List[NodeAStar] = [start_node]
    best_cost = {start: 0.0}
    visited = set()

    moves = [
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    ]

    while open_list:
        current = heapq.heappop(open_list)

        if current.position in visited:
            continue

        visited.add(current.position)

        if current.position == goal:
            path: List[GridPoint] = []
            node: Optional[NodeAStar] = current

            while node is not None:
                path.append(node.position)
                node = node.parent

            return list(reversed(path))

        cy, cx = current.position

        for dy, dx, step_cost in moves:
            ny = cy + dy
            nx = cx + dx

            if not (0 <= ny < height and 0 <= nx < width):
                continue

            if map_data[ny, nx] != 0:
                continue

            if dy != 0 and dx != 0:
                if map_data[cy, nx] != 0 or map_data[ny, cx] != 0:
                    continue

            extra_cost = 0.0
            if cost_map is not None:
                extra_cost = float(cost_map[ny, nx])

            next_position = (ny, nx)
            new_g = current.g + step_cost + extra_cost

            if new_g >= best_cost.get(next_position, float("inf")):
                continue

            h = math.hypot(ny - goal[0], nx - goal[1])
            best_cost[next_position] = new_g

            heapq.heappush(
                open_list,
                NodeAStar(
                    f=new_g + h,
                    h=h,
                    position=next_position,
                    g=new_g,
                    parent=current,
                ),
            )

    return None


class AirCleanPureController(Node):
    def __init__(self):
        super().__init__("air_clean_pure_controller")

        self.declare_parameter("command_topic", "/air_clean_command")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("path_topic", "/astar_path")
        self.declare_parameter("debug_topic", "/air_clean_debug")
        self.declare_parameter("shoe_detected_topic", "/shoe_detected")
        self.declare_parameter("scan_topic", "/scan")

        self.declare_parameter("aruco_image_topic", "/image_raw/compressed")
        self.declare_parameter("aruco_marker_id", 0)
        self.declare_parameter("aruco_alignment_enabled", True)

        # ArUco 정렬 안정화 파라미터
        self.declare_parameter("aruco_center_tolerance_px", 35)
        self.declare_parameter("aruco_angular_kp", 0.28)
        self.declare_parameter("aruco_max_angular_speed", 0.18)
        self.declare_parameter("aruco_lost_timeout", 1.2)
        self.declare_parameter("aruco_search_angular_speed", 0.08)
        self.declare_parameter("aruco_finish_required_frames", 5)
        self.declare_parameter("aruco_center_filter_alpha", 0.35)
        self.declare_parameter("aruco_min_angular_speed", 0.04)

        # ArUco 기준 중앙 보정값(px)
        # 0.0이면 화면 정중앙에 맞춤.
        # 로봇이 오른쪽/왼쪽으로 치우쳐 멈추면 이 값을 -40 ~ +40 범위에서 조절.
        # 예: 20.0 또는 -20.0
        self.declare_parameter("aruco_center_offset_px", 0.0)

        # 카메라가 좌우 반전되어 있거나 장착 방향이 반대면 True로 바꾸면 됨
        self.declare_parameter("aruco_reverse_angular_direction", False)

        self.declare_parameter("node_1_x", 1.0)
        self.declare_parameter("node_1_y", 0.0)
        self.declare_parameter("node_1_yaw", 0.0)

        self.declare_parameter("node_2_x", 0.0)
        self.declare_parameter("node_2_y", 0.0)
        self.declare_parameter("node_2_yaw", 0.0)

        self.declare_parameter("node_3_x", 0.0)
        self.declare_parameter("node_3_y", 0.0)
        self.declare_parameter("node_3_yaw", 0.0)

        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)

        self.declare_parameter("occupancy_threshold", 50)
        self.declare_parameter("inflation_radius_cells", 5)
        self.declare_parameter("use_scan_obstacles", True)
        self.declare_parameter("dynamic_obstacle_inflation_cells", 3)
        self.declare_parameter("dynamic_obstacle_max_range", 1.5)
        self.declare_parameter("dynamic_replan_interval_sec", 0.4)
        self.declare_parameter("replan_retry_interval_sec", 1.0)
        self.declare_parameter("control_frequency", 10.0)

        self.declare_parameter("linear_speed", 0.04)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("heading_gain", 1.0)
        self.declare_parameter("rotate_min_angular_speed", 0.20)
        self.declare_parameter("goal_tolerance", 0.12)
        self.declare_parameter("goal_yaw_tolerance", 0.02)
        self.declare_parameter("waypoint_tolerance", 0.08)
        self.declare_parameter("rotate_in_place_threshold", 0.55)
        self.declare_parameter("lookahead_points", 1)
        self.declare_parameter("lookahead_distance", 0.22)
        self.declare_parameter("path_index_max_advance_cells", 5)
        self.declare_parameter("path_smoothing_enabled", True)
        self.declare_parameter("safety_cost_radius_cells", 8)
        self.declare_parameter("safety_cost_weight", 2.5)

        self.declare_parameter("auto_clean_on_arrival", True)
        self.declare_parameter("mqtt_broker", "192.168.0.55")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_command_topic", "robot/1/cmd")

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.shoe_detected_topic = str(self.get_parameter("shoe_detected_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)

        self.aruco_image_topic = str(self.get_parameter("aruco_image_topic").value)
        self.aruco_marker_id = int(self.get_parameter("aruco_marker_id").value)
        self.aruco_alignment_enabled = bool(
            self.get_parameter("aruco_alignment_enabled").value
        )
        self.aruco_center_tolerance_px = int(
            self.get_parameter("aruco_center_tolerance_px").value
        )
        self.aruco_angular_kp = float(self.get_parameter("aruco_angular_kp").value)
        self.aruco_max_angular_speed = float(
            self.get_parameter("aruco_max_angular_speed").value
        )
        self.aruco_lost_timeout = float(
            self.get_parameter("aruco_lost_timeout").value
        )
        self.aruco_search_angular_speed = float(
            self.get_parameter("aruco_search_angular_speed").value
        )
        self.aruco_finish_required_frames = int(
            self.get_parameter("aruco_finish_required_frames").value
        )
        self.aruco_center_filter_alpha = float(
            self.get_parameter("aruco_center_filter_alpha").value
        )
        self.aruco_min_angular_speed = float(
            self.get_parameter("aruco_min_angular_speed").value
        )
        self.aruco_center_offset_px = float(
            self.get_parameter("aruco_center_offset_px").value
        )
        self.aruco_reverse_angular_direction = bool(
            self.get_parameter("aruco_reverse_angular_direction").value
        )

        self.auto_clean_on_arrival = bool(
            self.get_parameter("auto_clean_on_arrival").value
        )
        self.mqtt_broker = str(self.get_parameter("mqtt_broker").value)
        self.mqtt_port = int(self.get_parameter("mqtt_port").value)
        self.mqtt_command_topic = str(self.get_parameter("mqtt_command_topic").value)

        self.targets = {
            "node 1": (
                float(self.get_parameter("node_1_x").value),
                float(self.get_parameter("node_1_y").value),
            ),
            "node 2": (
                float(self.get_parameter("node_2_x").value),
                float(self.get_parameter("node_2_y").value),
            ),
            "node 3": (
                float(self.get_parameter("node_3_x").value),
                float(self.get_parameter("node_3_y").value),
            ),
            "home": (
                float(self.get_parameter("home_x").value),
                float(self.get_parameter("home_y").value),
            ),
        }
        self.target_yaws = {
            "node 1": float(self.get_parameter("node_1_yaw").value),
            "node 2": float(self.get_parameter("node_2_yaw").value),
            "node 3": float(self.get_parameter("node_3_yaw").value),
            "home": float(self.get_parameter("home_yaw").value),
        }

        self.occupancy_threshold = int(self.get_parameter("occupancy_threshold").value)
        self.inflation_radius_cells = int(self.get_parameter("inflation_radius_cells").value)
        self.use_scan_obstacles = bool(self.get_parameter("use_scan_obstacles").value)

        self.dynamic_obstacle_inflation_cells = int(
            self.get_parameter("dynamic_obstacle_inflation_cells").value
        )
        self.dynamic_obstacle_max_range = float(
            self.get_parameter("dynamic_obstacle_max_range").value
        )
        self.dynamic_replan_interval_sec = float(
            self.get_parameter("dynamic_replan_interval_sec").value
        )
        self.replan_retry_interval_sec = float(
            self.get_parameter("replan_retry_interval_sec").value
        )
        self.control_frequency = float(self.get_parameter("control_frequency").value)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.rotate_min_angular_speed = float(
            self.get_parameter("rotate_min_angular_speed").value
        )
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.goal_yaw_tolerance = float(
            self.get_parameter("goal_yaw_tolerance").value
        )
        self.waypoint_tolerance = float(self.get_parameter("waypoint_tolerance").value)
        self.rotate_in_place_threshold = float(
            self.get_parameter("rotate_in_place_threshold").value
        )
        self.lookahead_points = int(self.get_parameter("lookahead_points").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.path_index_max_advance_cells = int(
            self.get_parameter("path_index_max_advance_cells").value
        )
        self.path_smoothing_enabled = bool(
            self.get_parameter("path_smoothing_enabled").value
        )

        self.safety_cost_radius_cells = int(
            self.get_parameter("safety_cost_radius_cells").value
        )
        self.safety_cost_weight = float(self.get_parameter("safety_cost_weight").value)

        self.map_data: Optional[np.ndarray] = None
        self.inflated_map: Optional[np.ndarray] = None
        self.safety_cost_map: Optional[np.ndarray] = None
        self.dynamic_obstacle_map: Optional[np.ndarray] = None

        self.map_resolution = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_origin: WorldPoint = (0.0, 0.0)
        self.map_frame = "map"

        self.current_pose: Optional[WorldPoint] = None
        self.current_yaw = 0.0

        self.goal_pose: Optional[WorldPoint] = None
        self.goal_yaw: Optional[float] = None
        self.goal_label = ""
        self.need_replan = False

        self.path_world: List[WorldPoint] = []
        self.path_grid: List[GridPoint] = []
        self.path_index = 0

        self.last_dynamic_replan_time = 0.0
        self.next_replan_time = 0.0
        self.last_control_log_time = 0.0
        self.last_aruco_log_time = 0.0

        self.shoe_detected = False
        self.clean_after_arrival = False

        # home 도착 후 ArUco 정렬 상태
        self.aruco_aligning = False
        self.aruco_target_center_x: Optional[float] = None
        self.aruco_raw_center_x: Optional[float] = None
        self.aruco_frame_width: Optional[int] = None
        self.aruco_last_seen_time = None
        self.aruco_stable_frames = 0
        self.aruco_last_error_px: Optional[float] = None
        self.aruco_last_search_direction = 1.0

        self.mqtt_client = None
        self.setup_mqtt_client()

        self.aruco_detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters(),
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            pose_qos,
        )

        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.create_subscription(
            Bool,
            self.shoe_detected_topic,
            self.shoe_detected_callback,
            10,
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        self.create_subscription(
            CompressedImage,
            self.aruco_image_topic,
            self.aruco_image_callback,
            qos_profile_sensor_data,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.create_timer(1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(
            f"AirClean A* + Pure Pursuit controller started. "
            f"command={self.command_topic}, map={self.map_topic}, "
            f"pose={self.pose_topic}, scan={self.scan_topic}, "
            f"cmd_vel={self.cmd_vel_topic}, path={self.path_topic}, "
            f"debug={self.debug_topic}"
        )

        self.get_logger().info(
            f"inflation_radius_cells={self.inflation_radius_cells}, "
            f"dynamic_obstacle_inflation_cells={self.dynamic_obstacle_inflation_cells}, "
            f"safety_cost_radius_cells={self.safety_cost_radius_cells}, "
            f"safety_cost_weight={self.safety_cost_weight}"
        )

        self.get_logger().info(
            f"ArUco home alignment: enabled={self.aruco_alignment_enabled}, "
            f"image={self.aruco_image_topic}, marker_id={self.aruco_marker_id}, "
            f"tolerance={self.aruco_center_tolerance_px}px, "
            f"center_offset={self.aruco_center_offset_px:.1f}px, "
            f"kp={self.aruco_angular_kp:.2f}, "
            f"max_w={self.aruco_max_angular_speed:.2f}, "
            f"search_w={self.aruco_search_angular_speed:.2f}, "
            f"lost_timeout={self.aruco_lost_timeout:.2f}, "
            f"required_frames={self.aruco_finish_required_frames}, "
            f"reverse_direction={self.aruco_reverse_angular_direction}"
        )

        self.get_logger().info(
            f"Auto CLEAN on arrival: enabled={self.auto_clean_on_arrival}, "
            f"mqtt={self.mqtt_broker}:{self.mqtt_port}, "
            f"topic={self.mqtt_command_topic}"
        )

        self.publish_debug(
            f"pure controller ready: cmd_vel={self.cmd_vel_topic}, "
            f"path={self.path_topic}, aruco_image={self.aruco_image_topic}, "
            f"aruco_offset={self.aruco_center_offset_px:.1f}px"
        )

    def setup_mqtt_client(self):
        if mqtt is None:
            self.get_logger().warning(
                "paho-mqtt is not installed. CLEAN MQTT publish is disabled. "
                "Install with: pip install paho-mqtt"
            )
            return

        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()

            self.get_logger().info(
                f"MQTT connected for CLEAN command: "
                f"{self.mqtt_broker}:{self.mqtt_port}, topic={self.mqtt_command_topic}"
            )

        except Exception as e:
            self.mqtt_client = None
            self.get_logger().warning(f"MQTT connect failed for CLEAN command: {e}")

    def publish_clean_command(self):
        if not self.auto_clean_on_arrival:
            self.get_logger().info("Auto CLEAN is disabled. Skip CLEAN publish.")
            self.publish_debug("auto clean disabled: skip CLEAN")
            return

        if self.mqtt_client is None:
            self.get_logger().warning("Cannot publish CLEAN: MQTT client is not connected")
            self.publish_debug("cannot publish CLEAN: mqtt disconnected")
            return

        try:
            self.mqtt_client.publish(self.mqtt_command_topic, "CLEAN")
            self.get_logger().info(
                f"Published CLEAN after arrival to MQTT topic {self.mqtt_command_topic}"
            )
            self.publish_debug("arrival reached: CLEAN published")

        except Exception as e:
            self.get_logger().warning(f"Failed to publish CLEAN command: {e}")
            self.publish_debug(f"failed CLEAN publish: {e}")

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()

        if command in ("1", "node1", "node_1", "node 1"):
            self.set_goal_from_target("node 1")

        elif command in ("2", "node2", "node_2", "node 2"):
            self.set_goal_from_target("node 2")

        elif command in ("3", "node3", "node_3", "node 3"):
            self.set_goal_from_target("node 3")

        elif command in ("go", "target", "dirty", "dirty_zone"):
            self.set_goal_from_target("node 1")

        elif command in ("home", "return", "back"):
            self.set_goal_from_target("home")

        elif command in ("stop", "x", "cancel"):
            self.cancel_navigation()

        elif command in ("clean", "clean_on", "clean_off"):
            self.get_logger().info(
                f'Received "{msg.data}" command. Navigation controller ignores cleaning command.'
            )

        else:
            self.get_logger().warning(
                f'Unknown command "{msg.data}". '
                "Use one of: node1, node2, node3, home, stop."
            )

    def set_goal_from_target(self, label: str):
        if label not in self.targets:
            self.get_logger().warning(f"Unknown target label: {label}")
            return

        self.reset_aruco_alignment()
        x, y = self.targets[label]
        self.goal_pose = (x, y)
        self.goal_yaw = self.target_yaws[label]
        self.goal_label = label
        self.need_replan = True

        self.clean_after_arrival = label in ("node 1", "node 2", "node 3")

        self.clear_path(keep_goal=True)

        self.get_logger().info(
            f"Received command for {label}. Goal set to x={x:.2f}, y={y:.2f}, "
            f"yaw={self.goal_yaw:.3f}, clean_after_arrival={self.clean_after_arrival}"
        )
        self.publish_debug(
            f"goal {label}: x={x:.2f}, y={y:.2f}, yaw={self.goal_yaw:.3f}, "
            f"clean={self.clean_after_arrival}"
        )

        self.plan_path()

    def goal_callback(self, msg: PoseStamped):
        self.reset_aruco_alignment()
        self.goal_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        self.goal_yaw = self.quaternion_to_yaw(msg.pose.orientation)
        self.goal_label = "external goal"
        self.need_replan = True

        self.clean_after_arrival = False

        self.clear_path(keep_goal=True)

        self.get_logger().info(
            f"Received /goal_pose. "
            f"Goal set to x={self.goal_pose[0]:.2f}, y={self.goal_pose[1]:.2f}, "
            f"yaw={self.goal_yaw:.3f}, clean_after_arrival={self.clean_after_arrival}"
        )
        self.publish_debug(
            f"external goal: x={self.goal_pose[0]:.2f}, y={self.goal_pose[1]:.2f}, "
            f"yaw={self.goal_yaw:.3f}"
        )

        self.plan_path()

    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = float(msg.info.resolution)
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)
        self.map_origin = (
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
        )
        self.map_frame = msg.header.frame_id or "map"

        grid = np.array(msg.data, dtype=np.int16).reshape(
            (self.map_height, self.map_width)
        )

        obstacle_mask = np.logical_or(
            grid < 0,
            grid >= self.occupancy_threshold,
        )

        self.map_data = np.where(obstacle_mask, 1, 0).astype(np.uint8)

        self.inflated_map = self.inflate_map(
            self.map_data,
            self.inflation_radius_cells,
        )

        self.safety_cost_map = self.build_safety_cost_map(self.map_data)
        self.dynamic_obstacle_map = np.zeros_like(self.map_data, dtype=np.uint8)

        self.get_logger().info(
            f"Map received: {self.map_width}x{self.map_height}, "
            f"resolution={self.map_resolution:.3f}, "
            f"inflation_radius_cells={self.inflation_radius_cells}"
        )
        self.publish_debug(
            f"map {self.map_width}x{self.map_height}, res={self.map_resolution:.3f}, "
            f"inflation={self.inflation_radius_cells}"
        )

        if self.goal_pose is not None and self.need_replan:
            self.plan_path()

    def scan_callback(self, msg: LaserScan):
        if not self.use_scan_obstacles:
            return

        if self.map_data is None or self.current_pose is None:
            return

        dynamic_map = np.zeros_like(self.map_data, dtype=np.uint8)

        angle = float(msg.angle_min)
        range_max = (
            float(msg.range_max)
            if math.isfinite(msg.range_max)
            else float("inf")
        )
        usable_max_range = min(range_max * 0.95, self.dynamic_obstacle_max_range)

        for distance in msg.ranges:
            if math.isfinite(distance):
                distance = float(distance)

                if msg.range_min <= distance <= usable_max_range:
                    obstacle_angle = self.current_yaw + angle
                    obstacle = (
                        self.current_pose[0]
                        + distance * math.cos(obstacle_angle),
                        self.current_pose[1]
                        + distance * math.sin(obstacle_angle),
                    )
                    cell = self.world_to_grid(obstacle)

                    if cell is not None:
                        dynamic_map[cell[0], cell[1]] = 1

            angle += float(msg.angle_increment)

        self.dynamic_obstacle_map = self.inflate_map(
            dynamic_map,
            self.dynamic_obstacle_inflation_cells,
        )

        if self.goal_pose is not None and self.path_world:
            now = self.get_clock().now().nanoseconds / 1_000_000_000.0

            if now - self.last_dynamic_replan_time >= self.dynamic_replan_interval_sec:
                self.last_dynamic_replan_time = now
                self.need_replan = True

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if self.goal_pose is not None and self.need_replan:
            self.plan_path()

    def shoe_detected_callback(self, msg: Bool):
        if self.shoe_detected == msg.data:
            return

        self.shoe_detected = bool(msg.data)

        if self.shoe_detected:
            self.publish_stop()
            self.get_logger().warning("Shoe detected. Stopping robot.")
            return

        self.get_logger().info("Shoe cleared. Resuming navigation.")

    def aruco_image_callback(self, msg: CompressedImage):
        if not self.aruco_aligning:
            return

        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("Failed to decode compressed image for ArUco")
            return

        height, width = frame.shape[:2]
        self.aruco_frame_width = width

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is None:
            return

        best_center_x = None
        best_area = -1.0

        for marker_corners, detected_id in zip(corners, ids.flatten()):
            if int(detected_id) != self.aruco_marker_id:
                continue

            points = marker_corners.reshape(-1, 2)
            center = points.mean(axis=0)

            area = abs(cv2.contourArea(points.astype(np.float32)))

            if area > best_area:
                best_area = area
                best_center_x = float(center[0])

        if best_center_x is None:
            return

        self.aruco_raw_center_x = best_center_x

        if self.aruco_target_center_x is None:
            self.aruco_target_center_x = best_center_x
        else:
            alpha = self.clamp01(self.aruco_center_filter_alpha)
            self.aruco_target_center_x = (
                alpha * best_center_x
                + (1.0 - alpha) * self.aruco_target_center_x
            )

        self.aruco_last_seen_time = self.get_clock().now()

        image_center_x = width / 2.0 + self.aruco_center_offset_px
        self.aruco_last_error_px = self.aruco_target_center_x - image_center_x

        if self.aruco_last_error_px > 0:
            self.aruco_last_search_direction = -1.0
        elif self.aruco_last_error_px < 0:
            self.aruco_last_search_direction = 1.0

    def plan_path(self):
        if self.goal_pose is None:
            return

        now = self.get_clock().now().nanoseconds / 1_000_000_000.0

        if now < self.next_replan_time:
            return

        if self.map_data is None or self.inflated_map is None:
            self.get_logger().warn("Map is not ready yet. Cannot plan path.")
            self.publish_debug("cannot plan: map not ready")
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        if self.current_pose is None:
            self.get_logger().warn("Robot pose is not ready yet. Cannot plan path.")
            self.publish_debug("cannot plan: robot pose not ready")
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        start = self.world_to_grid(self.current_pose)
        goal = self.world_to_grid(self.goal_pose)

        if start is None:
            self.get_logger().warn("Start pose is outside the map.")
            self.publish_debug("cannot plan: start outside map")
            self.clear_path(keep_goal=True)
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        if goal is None:
            self.get_logger().warn("Goal pose is outside the map.")
            self.publish_debug("cannot plan: goal outside map")
            self.clear_path(keep_goal=True)
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        planning_map = self.get_planning_map().copy()

        self.get_logger().info(
            f"planning {self.goal_label}: start={start}, goal={goal}, "
            f"start_blocked={int(planning_map[start[0], start[1]])}, "
            f"goal_blocked={int(planning_map[goal[0], goal[1]])}"
        )
        self.publish_debug(
            f"planning {self.goal_label}: start={start}, goal={goal}, "
            f"blocked=({int(planning_map[start[0], start[1]])},"
            f"{int(planning_map[goal[0], goal[1]])})"
        )

        self.clear_cell_area(planning_map, start, radius=1)
        self.clear_cell_area(planning_map, goal, radius=1)

        path_grid = run_astar(
            planning_map,
            start,
            goal,
            self.map_width,
            self.map_height,
            cost_map=self.safety_cost_map,
        )

        if not path_grid:
            self.get_logger().warn(
                f"A* could not find a path to {self.goal_label}. "
                f"start={start}, goal={goal}. "
                f"Try reducing inflation_radius_cells if this keeps happening."
            )
            self.publish_debug(
                f"path failed: {self.goal_label}, start={start}, goal={goal}"
            )
            self.clear_path(keep_goal=True)
            self.publish_stop()
            self.need_replan = True
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        raw_path_len = len(path_grid)
        if self.path_smoothing_enabled:
            path_grid = self.smooth_path_grid(path_grid, planning_map)

        self.path_grid = path_grid
        self.path_world = [self.grid_to_world(point) for point in path_grid]
        self.path_index = 0
        self.need_replan = False
        self.next_replan_time = 0.0

        self.publish_path()

        self.get_logger().info(
            f"A* path planned to {self.goal_label}: "
            f"{len(self.path_world)} waypoints, raw={raw_path_len}"
        )
        self.publish_debug(
            f"path planned: {self.goal_label}, waypoints={len(self.path_world)}, "
            f"raw={raw_path_len}"
        )

    def control_loop(self):
        if self.shoe_detected:
            self.publish_stop()
            return

        if self.aruco_aligning:
            self.control_aruco_alignment()
            return

        if self.goal_pose is None:
            return

        if self.current_pose is None:
            return

        if self.need_replan:
            self.plan_path()
            return

        goal_distance = self.distance(self.current_pose, self.goal_pose)

        if goal_distance <= self.goal_tolerance:
            if self.goal_yaw is not None:
                yaw_error = self.normalize_angle(self.goal_yaw - self.current_yaw)

                if abs(yaw_error) > self.goal_yaw_tolerance:
                    self.cmd_pub.publish(self.compute_goal_yaw_cmd(yaw_error))
                    return

            reached_label = self.goal_label
            should_clean = self.clean_after_arrival

            self.publish_stop()

            if reached_label == "home" and self.aruco_alignment_enabled:
                self.get_logger().info(
                    "Home reached. Start ArUco marker alignment."
                )
                self.publish_debug("home reached: start aruco alignment")
                self.clean_after_arrival = False
                self.start_aruco_alignment()
                return

            self.get_logger().info(
                f"Goal reached: {reached_label}, clean_after_arrival={should_clean}"
            )
            self.publish_debug(
                f"goal reached: {reached_label}, clean={should_clean}"
            )

            if should_clean:
                self.publish_clean_command()
                self.clean_after_arrival = False

            self.clear_path(keep_goal=False)
            self.goal_pose = None
            self.goal_yaw = None
            self.goal_label = ""
            return

        if not self.path_world:
            self.need_replan = True
            return

        self.advance_path_index()

        if self.path_index >= len(self.path_world):
            self.publish_stop()
            self.need_replan = True
            return

        target = self.select_tracking_target()

        twist, heading, heading_error, distance_to_target = (
            self.compute_pure_pursuit_cmd(target, goal_distance)
        )

        self.cmd_pub.publish(twist)

        self.log_control_state(
            target=target,
            distance_to_target=distance_to_target,
            heading=heading,
            heading_error=heading_error,
            twist=twist,
            goal_distance=goal_distance,
        )

    def start_aruco_alignment(self):
        self.publish_stop()
        self.clear_path(keep_goal=False)
        self.goal_pose = None
        self.goal_label = ""
        self.need_replan = False

        self.aruco_aligning = True
        self.aruco_target_center_x = None
        self.aruco_raw_center_x = None
        self.aruco_frame_width = None
        self.aruco_last_seen_time = None
        self.aruco_stable_frames = 0
        self.aruco_last_error_px = None
        self.aruco_last_search_direction = 1.0
        self.last_aruco_log_time = 0.0

        self.get_logger().info(
            f"ArUco alignment started. target_id={self.aruco_marker_id}, "
            f"center_offset={self.aruco_center_offset_px:.1f}px"
        )
        self.publish_debug(
            f"aruco alignment started: id={self.aruco_marker_id}, "
            f"offset={self.aruco_center_offset_px:.1f}px"
        )

    def reset_aruco_alignment(self):
        if self.aruco_aligning:
            self.publish_stop()

        self.aruco_aligning = False
        self.aruco_target_center_x = None
        self.aruco_raw_center_x = None
        self.aruco_frame_width = None
        self.aruco_last_seen_time = None
        self.aruco_stable_frames = 0
        self.aruco_last_error_px = None

    def control_aruco_alignment(self):
        twist = Twist()

        if (
            self.aruco_target_center_x is None
            or self.aruco_frame_width is None
            or not self.is_aruco_marker_recent()
        ):
            self.aruco_stable_frames = 0

            search_direction = self.aruco_last_search_direction
            if self.aruco_reverse_angular_direction:
                search_direction *= -1.0

            twist.linear.x = 0.0
            twist.angular.z = self.clamp(
                search_direction * abs(self.aruco_search_angular_speed),
                self.aruco_max_angular_speed,
            )
            self.cmd_pub.publish(twist)
            self.log_aruco_state(
                f"searching marker: angular_z={twist.angular.z:.3f}"
            )
            return

        image_center_x = self.aruco_frame_width / 2.0 + self.aruco_center_offset_px
        error_px = self.aruco_target_center_x - image_center_x
        self.aruco_last_error_px = error_px

        if abs(error_px) <= self.aruco_center_tolerance_px:
            self.aruco_stable_frames += 1
            self.publish_stop()

            self.log_aruco_state(
                f"aruco center stable: "
                f"raw_x={self.aruco_raw_center_x}, "
                f"filtered_x={self.aruco_target_center_x:.1f}, "
                f"target_center={image_center_x:.1f}, "
                f"offset={self.aruco_center_offset_px:.1f}, "
                f"error={error_px:.1f}px, "
                f"frames={self.aruco_stable_frames}/{self.aruco_finish_required_frames}"
            )

            if self.aruco_stable_frames >= self.aruco_finish_required_frames:
                self.publish_stop()
                self.aruco_aligning = False
                self.aruco_target_center_x = None
                self.aruco_raw_center_x = None
                self.aruco_frame_width = None
                self.aruco_last_seen_time = None
                self.aruco_stable_frames = 0

                self.get_logger().info(
                    f"ArUco aligned. Error={error_px:.1f}px, "
                    f"offset={self.aruco_center_offset_px:.1f}px. Robot stopped."
                )
                self.publish_debug(
                    f"aruco aligned: error={error_px:.1f}px, "
                    f"offset={self.aruco_center_offset_px:.1f}px"
                )
            return

        self.aruco_stable_frames = 0

        normalized_error = error_px / max(1.0, image_center_x)

        # 일반 카메라 기준:
        # 마커가 화면 오른쪽이면 error > 0, 로봇은 오른쪽으로 돌아야 하므로 angular_z는 음수.
        direction_sign = -1.0

        # 카메라 화면이 좌우 반전되어 있거나 장착 방향이 반대면 방향 반전.
        if self.aruco_reverse_angular_direction:
            direction_sign *= -1.0

        angular_z = direction_sign * self.aruco_angular_kp * normalized_error
        angular_z = self.clamp(angular_z, self.aruco_max_angular_speed)

        if abs(angular_z) < self.aruco_min_angular_speed:
            angular_z = math.copysign(self.aruco_min_angular_speed, angular_z)

        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        self.log_aruco_state(
            f"aligning marker: raw_x={self.aruco_raw_center_x}, "
            f"filtered_x={self.aruco_target_center_x:.1f}, "
            f"target_center={image_center_x:.1f}, "
            f"offset={self.aruco_center_offset_px:.1f}, "
            f"error={error_px:.1f}px, angular_z={angular_z:.3f}, "
            f"reverse={self.aruco_reverse_angular_direction}"
        )

    def is_aruco_marker_recent(self) -> bool:
        if self.aruco_last_seen_time is None:
            return False

        elapsed = (
            self.get_clock().now() - self.aruco_last_seen_time
        ).nanoseconds / 1e9

        return elapsed <= self.aruco_lost_timeout

    def log_aruco_state(self, text: str):
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0

        if now - self.last_aruco_log_time < 1.0:
            return

        self.last_aruco_log_time = now
        self.get_logger().info(text)
        self.publish_debug(text)

    def advance_path_index(self):
        if self.current_pose is None or not self.path_world:
            return

        last_index = len(self.path_world) - 1
        max_advance = max(1, self.path_index_max_advance_cells)
        search_end = min(last_index, self.path_index + max_advance)

        best_index = self.path_index
        best_distance = float("inf")

        for index in range(self.path_index, search_end + 1):
            waypoint = self.path_world[index]
            distance_to_waypoint = self.distance(self.current_pose, waypoint)

            if distance_to_waypoint < best_distance:
                best_distance = distance_to_waypoint
                best_index = index

        if best_index > self.path_index:
            self.path_index = best_index

        while self.path_index < last_index:
            waypoint = self.path_world[self.path_index]

            if self.distance(self.current_pose, waypoint) > self.waypoint_tolerance:
                break

            self.path_index += 1

    def select_tracking_target(self) -> WorldPoint:
        if self.current_pose is None:
            return self.goal_pose if self.goal_pose is not None else (0.0, 0.0)

        if not self.path_world:
            return self.goal_pose if self.goal_pose is not None else self.current_pose

        last_index = len(self.path_world) - 1

        if self.goal_pose is not None:
            goal_distance = self.distance(self.current_pose, self.goal_pose)

            if goal_distance <= max(self.goal_tolerance * 2.0, self.lookahead_distance):
                return self.goal_pose

        start_index = max(0, min(self.path_index, last_index))

        if start_index >= last_index:
            return self.path_world[last_index]

        accumulated_distance = 0.0
        previous_point = self.path_world[start_index]

        target_index = last_index

        for index in range(start_index + 1, len(self.path_world)):
            point = self.path_world[index]
            accumulated_distance += self.distance(previous_point, point)
            previous_point = point

            if accumulated_distance >= self.lookahead_distance:
                target_index = index
                break

        return self.path_world[target_index]

    def compute_pure_pursuit_cmd(
        self,
        target: WorldPoint,
        goal_distance: float,
    ) -> Tuple[Twist, float, float, float]:
        distance_to_target = self.distance(self.current_pose, target)

        heading = math.atan2(
            target[1] - self.current_pose[1],
            target[0] - self.current_pose[0],
        )

        heading_error = self.normalize_angle(heading - self.current_yaw)

        lookahead = max(
            self.lookahead_distance,
            distance_to_target,
            0.05,
        )

        linear_x = min(self.linear_speed, goal_distance)

        if goal_distance < 0.35:
            linear_x = min(linear_x, max(0.025, goal_distance * 0.45))

        abs_heading_error = abs(heading_error)

        if abs_heading_error > self.rotate_in_place_threshold:
            linear_x = 0.0
            angular_z = self.heading_gain * heading_error
        else:
            if abs_heading_error > 0.45:
                linear_x = min(linear_x, self.linear_speed * 0.45)
            elif abs_heading_error > 0.25:
                linear_x = min(linear_x, self.linear_speed * 0.70)

            angular_z = linear_x * (2.0 * math.sin(heading_error)) / lookahead

        twist = Twist()
        twist.linear.x = max(0.0, linear_x)
        if twist.linear.x == 0.0 and abs(angular_z) > 0.0:
            angular_z = math.copysign(
                max(abs(angular_z), self.rotate_min_angular_speed),
                angular_z,
            )
        twist.angular.z = self.clamp(
            angular_z,
            self.max_angular_speed,
        )

        return twist, heading, heading_error, distance_to_target

    def compute_goal_yaw_cmd(self, yaw_error: float) -> Twist:
        angular_z = self.heading_gain * yaw_error
        angular_z = math.copysign(
            max(abs(angular_z), self.rotate_min_angular_speed),
            angular_z,
        )

        twist = Twist()
        twist.angular.z = self.clamp(angular_z, self.max_angular_speed)
        return twist

    def log_control_state(
        self,
        target: WorldPoint,
        distance_to_target: float,
        heading: float,
        heading_error: float,
        twist: Twist,
        goal_distance: float,
    ):
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0

        if now - self.last_control_log_time < 1.0:
            return

        self.last_control_log_time = now

        text = (
            f"control pose=({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}), "
            f"yaw={self.current_yaw:.2f}, path_index={self.path_index}, "
            f"target=({target[0]:.2f}, {target[1]:.2f}), "
            f"distance_to_target={distance_to_target:.2f}, "
            f"heading={heading:.2f}, heading_error={heading_error:.2f}, "
            f"cmd=({twist.linear.x:.2f}, {twist.angular.z:.2f}), "
            f"goal_distance={goal_distance:.2f}"
        )
        self.get_logger().info(text)
        self.publish_debug(text)

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        for x, y in self.path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_debug(self, text: str):
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)

    def cancel_navigation(self):
        self.publish_stop()
        self.reset_aruco_alignment()
        self.clear_path(keep_goal=False)
        self.goal_pose = None
        self.goal_yaw = None
        self.goal_label = ""
        self.need_replan = False
        self.clean_after_arrival = False
        self.get_logger().info("Navigation canceled by stop command.")

    def clear_path(self, keep_goal: bool = True):
        self.path_world = []
        self.path_grid = []
        self.path_index = 0

        if not keep_goal:
            self.goal_pose = None
            self.goal_yaw = None

        self.publish_path()

    def world_to_grid(self, position: WorldPoint) -> Optional[GridPoint]:
        if self.map_resolution <= 0.0:
            return None

        mx = math.floor((position[0] - self.map_origin[0]) / self.map_resolution)
        my = math.floor((position[1] - self.map_origin[1]) / self.map_resolution)

        if 0 <= my < self.map_height and 0 <= mx < self.map_width:
            return my, mx

        return None

    def get_planning_map(self) -> np.ndarray:
        if self.inflated_map is None:
            raise RuntimeError("Inflated map is not ready")

        if not self.use_scan_obstacles or self.dynamic_obstacle_map is None:
            return self.inflated_map

        return np.maximum(self.inflated_map, self.dynamic_obstacle_map)

    def build_safety_cost_map(self, map_data: np.ndarray) -> np.ndarray:
        cost_map = np.zeros_like(map_data, dtype=np.float32)

        radius = max(0, int(self.safety_cost_radius_cells))
        weight = max(0.0, float(self.safety_cost_weight))

        if radius <= 0 or weight <= 0.0:
            return cost_map

        obstacle_points = np.argwhere(map_data != 0)

        for y, x in obstacle_points:
            y_min = max(0, y - radius)
            y_max = min(map_data.shape[0], y + radius + 1)
            x_min = max(0, x - radius)
            x_max = min(map_data.shape[1], x + radius + 1)

            for yy in range(y_min, y_max):
                for xx in range(x_min, x_max):
                    distance_cells = math.hypot(yy - y, xx - x)

                    if distance_cells > radius:
                        continue

                    if map_data[yy, xx] != 0:
                        continue

                    cost = weight * (1.0 - distance_cells / float(radius))

                    if cost > cost_map[yy, xx]:
                        cost_map[yy, xx] = cost

        return cost_map

    def grid_to_world(self, cell: GridPoint) -> WorldPoint:
        y, x = cell

        wx = self.map_origin[0] + (x + 0.5) * self.map_resolution
        wy = self.map_origin[1] + (y + 0.5) * self.map_resolution

        return wx, wy

    def smooth_path_grid(
        self,
        path: List[GridPoint],
        planning_map: np.ndarray,
    ) -> List[GridPoint]:
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        anchor_index = 0

        while anchor_index < len(path) - 1:
            next_index = len(path) - 1

            while next_index > anchor_index + 1:
                if self.has_line_of_sight(
                    path[anchor_index],
                    path[next_index],
                    planning_map,
                ):
                    break

                next_index -= 1

            smoothed.append(path[next_index])
            anchor_index = next_index

        return smoothed

    @staticmethod
    def has_line_of_sight(
        start: GridPoint,
        goal: GridPoint,
        planning_map: np.ndarray,
    ) -> bool:
        y0, x0 = start
        y1, x1 = goal
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if planning_map[y0, x0] != 0:
                return False

            if y0 == y1 and x0 == x1:
                return True

            err2 = 2 * err

            if err2 > -dy:
                err -= dy
                x0 += sx

            if err2 < dx:
                err += dx
                y0 += sy

            if not (0 <= y0 < planning_map.shape[0] and 0 <= x0 < planning_map.shape[1]):
                return False

    @staticmethod
    def inflate_map(map_data: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return map_data.copy()

        inflated = map_data.copy()
        obstacle_points = np.argwhere(map_data != 0)
        radius_sq = radius * radius

        for y, x in obstacle_points:
            y_min = max(0, y - radius)
            y_max = min(map_data.shape[0], y + radius + 1)
            x_min = max(0, x - radius)
            x_max = min(map_data.shape[1], x + radius + 1)

            for yy in range(y_min, y_max):
                for xx in range(x_min, x_max):
                    dy = yy - y
                    dx = xx - x

                    if dy * dy + dx * dx <= radius_sq:
                        inflated[yy, xx] = 1

        return inflated

    @staticmethod
    def clear_cell_area(map_data: np.ndarray, center: GridPoint, radius: int):
        cy, cx = center

        y_min = max(0, cy - radius)
        y_max = min(map_data.shape[0], cy + radius + 1)
        x_min = max(0, cx - radius)
        x_max = min(map_data.shape[1], cx + radius + 1)

        map_data[y_min:y_max, x_min:x_max] = 0

    @staticmethod
    def distance(a: Sequence[float], b: Sequence[float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    @staticmethod
    def clamp01(value: float) -> float:
        return max(0.0, min(1.0, value))

    def destroy_node(self):
        if self.mqtt_client is not None:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirCleanPureController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
