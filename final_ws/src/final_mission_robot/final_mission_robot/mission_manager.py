#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


LOAD_POINT_1 = {
    "name": "LOAD_POINT_1",
    "x": 2.068,
    "y": -0.070,
    "yaw": 0.035,
    "qz": 0.017484554994104,
    "qw": 0.9998471334842433,
}

RIGHT_WALL_WAYPOINT_1 = {
    "name": "RIGHT_WALL_WAYPOINT_1",
    "x": 2.15,
    "y": -1.20,
    "yaw": -1.5708,
    "qz": -0.7071,
    "qw": 0.7071,
}

RIGHT_WALL_WAYPOINT_2 = {
    "name": "RIGHT_WALL_WAYPOINT_2",
    "x": 2.15,
    "y": -2.35,
    "yaw": -1.5708,
    "qz": -0.7071,
    "qw": 0.7071,
}

LOAD_POINT_2 = {
    "name": "LOAD_POINT_2",
    "x": 1.088,
    "y": -2.584,
    "yaw": -3.133,
    "qz": -0.9999897887805898,
    "qw": 0.004519107716300417,
}

F1_WAIT_POINT_AFTER_LOAD_1 = {
    "name": "F1_WAIT_POINT_AFTER_LOAD_1",
    "x": 1.091,
    "y": -1.128,
    "yaw": -1.514,
    "qz": -0.6868610836458342,
    "qw": 0.7267887256781508,
}

F2_LOAD2_APPROACH_POINT = {
    "name": "F2_LOAD2_APPROACH_POINT",
    "x": 1.929,
    "y": -2.668,
    "yaw": 3.14159,
    "qz": 1.0,
    "qw": 0.0,
}

TB3_HOME_POINT = {
    "name": "TB3_HOME_POINT",
    "x": -0.037569766737571876,
    "y": 0.10523828207830883,
    "yaw": -0.007148110090032988,
    "qz": -0.003574047428292722,
    "qw": 0.9999936120847033,
}

F1_HOME_POINT = {
    "name": "F1_HOME_POINT",
    "x": 0.128,
    "y": -1.112,
    "yaw": -1.5708,
    "qz": -0.7071,
    "qw": 0.7071,
}

F2_HOME_POINT = {
    "name": "F2_HOME_POINT",
    "x": -0.048,
    "y": -2.668,
    "yaw": -1.5708,
    "qz": -0.7071,
    "qw": 0.7071,
}

TB3_LOAD2_ROUTE = [
    RIGHT_WALL_WAYPOINT_1,
    RIGHT_WALL_WAYPOINT_2,
    LOAD_POINT_2,
]

F2_HOME_RETURN_ROUTE = [
    {
        "name": "F2_HOME_RETURN_WP1",
        "x": 0.650,
        "y": -2.400,
        "yaw": -1.5708,
        "qz": -0.7071,
        "qw": 0.7071,
    },
    {
        "name": "F2_HOME_RETURN_WP2",
        "x": 0.650,
        "y": -2.100,
        "yaw": -1.5708,
        "qz": -0.7071,
        "qw": 0.7071,
    },
    {
        "name": "F2_HOME_RETURN_WP3",
        "x": 0.150,
        "y": -2.100,
        "yaw": -1.5708,
        "qz": -0.7071,
        "qw": 0.7071,
    },
    {
        "name": "F2_HOME_RETURN_WP4",
        "x": 0.050,
        "y": -2.450,
        "yaw": -1.5708,
        "qz": -0.7071,
        "qw": 0.7071,
    },
    F2_HOME_POINT,
]


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.state = "IDLE"

        self.f1_pose_ok = False
        self.f1_pose_source = ""
        self.f1_last_relative_pose = ""
        self.f1_last_hybrid_status = ""

        self.f2_pose_ok = False
        self.f2_x = 0.0
        self.f2_y = 0.0
        self.f2_yaw = 0.0
        self.f2_last_relative_pose = ""

        self.tb3_pose_ok = False
        self.tb3_x = 0.0
        self.tb3_y = 0.0
        self.tb3_yaw = 0.0

        self.tb3_route_active = False
        self.tb3_route = []
        self.tb3_route_index = 0
        self.tb3_current_goal = None
        self.tb3_goal_reached_distance = 0.25
        self.tb3_goal_republish_period_sec = 3.0
        self.tb3_goal_last_pub_time = 0.0

        self.home_return_active = False
        self.home_goal_republish_period_sec = 3.0
        self.home_goal_last_pub_time = 0.0

        self.f2_home_route_active = False
        self.f2_home_route = []
        self.f2_home_route_index = 0
        self.f2_home_current_goal = None
        self.f2_home_goal_reached_distance = 0.25
        self.f2_home_goal_last_pub_time = 0.0

        self.formation_wait_start_time = None
        self.formation_wait_sec = 3.0

        self.f2_astar_start_time = None
        self.f2_astar_goal_republish_period_sec = 2.0
        self.f2_astar_goal_last_pub_time = 0.0
        self.f2_astar_wait_sec = 2.0

        self.load_exit_direction = "e"
        self.load_exit_start_time = None
        self.load_exit_last_cmd_time = 0.0
        self.load_exit_rotate_sec = 2.0
        self.load_exit_cmd_period_sec = 0.30
        self.load_exit_wait_pose_after_rotate_sec = 0.50

        self.mission_cmd_sub = self.create_subscription(
            String,
            "/mission_cmd",
            self.mission_cmd_callback,
            10,
        )

        self.f1_relative_pose_sub = self.create_subscription(
            String,
            "/f1/relative_pose",
            self.f1_relative_pose_callback,
            10,
        )

        self.f1_hybrid_status_sub = self.create_subscription(
            String,
            "/f1/hybrid_status",
            self.f1_hybrid_status_callback,
            10,
        )

        self.f2_relative_pose_sub = self.create_subscription(
            String,
            "/f2/relative_pose",
            self.f2_relative_pose_callback,
            10,
        )

        self.tb3_amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.tb3_amcl_pose_callback,
            10,
        )

        self.tb3_goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10,
        )

        self.f1_mode_pub = self.create_publisher(
            String,
            "/f1/mode",
            10,
        )

        self.f1_goal_pub = self.create_publisher(
            PoseStamped,
            "/f1/goal_pose",
            10,
        )

        self.f1_robot_cmd_pub = self.create_publisher(
            String,
            "/f1/robot_cmd",
            10,
        )

        self.f1_load_exit_cmd_pub = self.create_publisher(
            String,
            "/f1/load_exit_cmd",
            10,
        )

        self.f2_mode_pub = self.create_publisher(
            String,
            "/f2/mode",
            10,
        )

        self.f2_target_marker_cmd_pub = self.create_publisher(
            String,
            "/f2/target_marker_cmd",
            10,
        )

        self.f2_goal_pub = self.create_publisher(
            PoseStamped,
            "/f2/goal_pose",
            10,
        )

        self.f2_astar_goal_pub = self.create_publisher(
            PoseStamped,
            "/f2/astar_goal",
            10,
        )

        self.mission_status_pub = self.create_publisher(
            String,
            "/mission_status",
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.get_logger().info("mission_manager_node started")
        self.publish_status("IDLE", "Mission manager ready")

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def publish_status(self, state, message):
        self.state = state

        msg = String()
        msg.data = f"MISSION,state={state},message={message}"

        self.mission_status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def mission_cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"mission_cmd received: {cmd}")

        if cmd == "start":
            self.start_mission()

        elif cmd == "load_done_f1":
            self.handle_load_done_f1()

        elif cmd in ["f1_home_done", "f1_wait_done"]:
            self.handle_f1_home_done()

        elif cmd in ["go_load2", "tb3_load2", "start_load2"]:
            self.start_load2_astar_mode()

        elif cmd in ["f2_follow_159", "follow_159"]:
            self.switch_f2_to_tb3_follow()

        elif cmd == "load_done_f2":
            self.handle_load_done_f2()

        elif cmd in ["home", "return_home", "go_home"]:
            self.start_home_return()

        elif cmd == "home_done":
            self.handle_home_done()

        elif cmd == "load_exit_e":
            self.start_f1_load_exit("e")

        elif cmd == "load_exit_q":
            self.start_f1_load_exit("q")

        elif cmd in ["pause", "stop", "emergency_stop", "all_stop"]:
            self.stop_mission(cmd)

        else:
            self.publish_status("ERROR", f"Unknown mission_cmd: {cmd}")

    def tb3_amcl_pose_callback(self, msg):
        self.tb3_x = float(msg.pose.pose.position.x)
        self.tb3_y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        self.tb3_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.tb3_pose_ok = True

    def f2_relative_pose_callback(self, msg):
        data = msg.data.strip()
        self.f2_last_relative_pose = data

        values = self.parse_key_value_text(data)

        if "x" in values and "y" in values:
            try:
                self.f2_x = float(values["x"])
                self.f2_y = float(values["y"])
                self.f2_pose_ok = True

                if "yaw" in values:
                    self.f2_yaw = float(values["yaw"])

            except ValueError:
                self.f2_pose_ok = False

    def parse_key_value_text(self, data):
        result = {}
        parts = data.split(",")

        for part in parts[1:]:
            if "=" not in part:
                continue

            key, value = part.split("=", 1)
            result[key.strip()] = value.strip()

        return result

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    def f1_relative_pose_callback(self, msg):
        data = msg.data.strip()
        self.f1_last_relative_pose = data

        lower = data.lower()

        pose_ok = False
        pose_source = ""

        if "source=tb3_marker159" in lower:
            pose_ok = True
            pose_source = "tb3_marker159"

        elif "aruco_accepted=1" in lower:
            pose_ok = True
            pose_source = "aruco_accepted"

        elif "marker_seen=1" in lower:
            if "source=stop" not in lower and "source=lost" not in lower:
                pose_ok = True
                pose_source = "marker_seen"

        self.f1_pose_ok = pose_ok
        self.f1_pose_source = pose_source

    def f1_hybrid_status_callback(self, msg):
        data = msg.data.strip()
        self.f1_last_hybrid_status = data

        lower = data.lower()

        if "aruco_accepted=1" in lower:
            self.f1_pose_ok = True
            self.f1_pose_source = "hybrid_aruco_accepted"

        if "aruco_accepted=0" in lower and "marker_seen=0" in lower:
            if self.f1_pose_source != "tb3_marker159":
                self.f1_pose_ok = False
                self.f1_pose_source = ""

        if "tb3_pose_fresh=1" in lower and "f1_pose_fresh=1" in lower:
            self.f1_pose_ok = True
            self.f1_pose_source = "hybrid_tb3_map_ready"

    def start_mission(self):
        self.home_return_active = False
        self.f2_home_route_active = False

        self.f1_pose_ok = False
        self.f1_pose_source = ""
        self.formation_wait_start_time = None
        self.f2_astar_start_time = None
        self.stop_tb3_route_tracking()

        self.publish_status(
            "FORMATION_WAIT",
            "Mission started. F1 follows TB3 marker 159, F2 follows F1 marker 158. Waiting before TB3 moves.",
        )

        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "follow",
        )

        self.publish_target_marker(
            self.f2_target_marker_cmd_pub,
            "f2",
            158,
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "follow",
        )

        self.formation_wait_start_time = self.now_sec()

    def process_formation_wait(self):
        if self.formation_wait_start_time is None:
            self.formation_wait_start_time = self.now_sec()
            return

        elapsed = self.now_sec() - self.formation_wait_start_time

        if elapsed < self.formation_wait_sec:
            return

        self.formation_wait_start_time = None

        self.publish_status(
            "GO_TO_LOAD_1",
            "Formation wait done. TB3 moving to LOAD_POINT_1.",
        )

        self.publish_tb3_goal(LOAD_POINT_1)

    def handle_load_done_f1(self):
        if self.state not in [
            "GO_TO_LOAD_1",
            "WAIT_LOAD_F1",
            "F1_LOADING",
            "F1_POSE_RECOVERY_FAILED",
        ]:
            self.get_logger().warn(
                f"load_done_f1 received while state={self.state}. "
                "Continuing anyway for manual test."
            )

        self.get_logger().warn(
            "load_done_f1 received. F2 stop first, then F1 load-exit starts."
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "stop",
        )

        self.start_f1_load_exit("e")

    def start_f1_load_exit(self, direction):
        if direction not in ["e", "q"]:
            direction = "e"

        self.load_exit_direction = direction
        self.load_exit_start_time = self.now_sec()
        self.load_exit_last_cmd_time = 0.0

        self.publish_status(
            "F1_LOAD_EXIT_ROTATE",
            f"F1 load-exit started. Rotating with command {direction}",
        )

        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "load_exit",
        )

        self.publish_f1_load_exit_cmd("s")

    def process_f1_load_exit_rotate(self):
        now = self.now_sec()

        if self.load_exit_start_time is None:
            self.load_exit_start_time = now

        elapsed = now - self.load_exit_start_time

        if elapsed >= self.load_exit_rotate_sec:
            self.publish_f1_load_exit_cmd("s")
            self.load_exit_start_time = now

            self.publish_status(
                "F1_LOAD_EXIT_SETTLE",
                "F1 load-exit rotation done. Stop and wait before home waypoint.",
            )

            return

        if now - self.load_exit_last_cmd_time >= self.load_exit_cmd_period_sec:
            self.publish_f1_load_exit_cmd(
                self.load_exit_direction
            )

            self.load_exit_last_cmd_time = now

    def process_f1_load_exit_settle(self):
        now = self.now_sec()

        if self.load_exit_start_time is None:
            self.load_exit_start_time = now

        elapsed = now - self.load_exit_start_time

        if elapsed < self.load_exit_wait_pose_after_rotate_sec:
            self.publish_f1_load_exit_cmd("s")
            return

        self.publish_f1_load_exit_cmd("s")

        if self.f1_pose_ok:
            self.get_logger().info(
                f"F1 pose is ready after load-exit. source={self.f1_pose_source}. "
                "Sending F1 to HOME pose."
            )
        else:
            self.get_logger().warn(
                "F1 pose is still not confirmed after load-exit. "
                "Sending HOME waypoint anyway for manual test. "
                f"last_relative_pose={self.f1_last_relative_pose}, "
                f"last_hybrid_status={self.f1_last_hybrid_status}"
            )

        self.send_f1_to_home_pose()

    def send_f1_to_home_pose(self):
        self.publish_status(
            "F1_TO_HOME",
            "F1 moving to HOME. F2 remains stopped.",
        )

        self.publish_f1_load_exit_cmd("s")
        self.publish_f1_robot_cmd("s")

        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "waypoint",
        )

        self.publish_f1_goal(F1_HOME_POINT)

    def handle_f1_home_done(self):
        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "stop",
        )

        self.start_load2_astar_mode()

    def start_load2_astar_mode(self):
        self.home_return_active = False
        self.f2_home_route_active = False

        self.f2_astar_start_time = self.now_sec()
        self.f2_astar_goal_last_pub_time = 0.0

        self.publish_status(
            "F2_ASTAR_TO_LOAD2_APPROACH",
            "F1 reached HOME. F2 switches to waypoint/A* and TB3 starts LOAD_POINT_2 route.",
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "stop",
        )

        self.publish_target_marker(
            self.f2_target_marker_cmd_pub,
            "f2",
            159,
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "waypoint",
        )

        self.publish_f2_astar_goal(F2_LOAD2_APPROACH_POINT)

        self.start_tb3_load2_route_without_forcing_f2_follow()

    def process_f2_astar_goal_republish(self):
        if self.f2_astar_start_time is None:
            return

        now = self.now_sec()

        if now - self.f2_astar_start_time < self.f2_astar_wait_sec:
            return

        if now - self.f2_astar_goal_last_pub_time < self.f2_astar_goal_republish_period_sec:
            return

        if self.state in [
            "F2_ASTAR_TO_LOAD2_APPROACH",
            "TB3_TO_LOAD2_WP1",
            "TB3_TO_LOAD2_WP2",
            "TB3_TO_LOAD2_FINAL",
            "TB3_TO_LOAD2_ROUTE",
        ]:
            self.publish_robot_mode(
                self.f2_mode_pub,
                "f2",
                "waypoint",
            )

            self.publish_f2_astar_goal(F2_LOAD2_APPROACH_POINT)

    def switch_f2_to_tb3_follow(self):
        self.home_return_active = False
        self.f2_home_route_active = False
        self.f2_astar_start_time = None

        self.publish_target_marker(
            self.f2_target_marker_cmd_pub,
            "f2",
            159,
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "follow",
        )

        self.publish_status(
            "F2_FINAL_MARKER_FOLLOW",
            "F2 switched to TB3 marker 159 for final loading alignment.",
        )

    def start_tb3_load2_route_without_forcing_f2_follow(self):
        self.tb3_route_active = True
        self.tb3_route = TB3_LOAD2_ROUTE
        self.tb3_route_index = 0
        self.tb3_current_goal = self.tb3_route[self.tb3_route_index]
        self.tb3_goal_last_pub_time = 0.0

        self.publish_status(
            "TB3_TO_LOAD2_WP1",
            "TB3 route started. F2 uses A* waypoint to approach point instead of following TB3 through wall route.",
        )

        self.publish_tb3_route_goal(force=True)

    def start_tb3_load2_route(self):
        self.start_load2_astar_mode()

    def handle_load_done_f2(self):
        self.get_logger().warn(
            "load_done_f2 received. Starting TB3 + F2 home return. F1 is already home."
        )

        self.start_home_return()

    def start_home_return(self):
        self.stop_tb3_route_tracking()
        self.formation_wait_start_time = None
        self.f2_astar_start_time = None

        self.home_return_active = True
        self.home_goal_last_pub_time = 0.0

        self.publish_f1_load_exit_cmd("s")
        self.publish_f1_robot_cmd("s")

        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "stop",
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "waypoint",
        )

        self.publish_tb3_goal(TB3_HOME_POINT)
        self.start_f2_home_return_route()

        self.home_goal_last_pub_time = self.now_sec()

        self.publish_status(
            "HOME_RETURNING",
            "F2 loading complete. TB3 returns home and F2 follows fixed home-return waypoints. F1 is already home.",
        )

    def start_f2_home_return_route(self):
        self.f2_home_route_active = True
        self.f2_home_route = F2_HOME_RETURN_ROUTE
        self.f2_home_route_index = 0
        self.f2_home_current_goal = self.f2_home_route[self.f2_home_route_index]
        self.f2_home_goal_last_pub_time = 0.0

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "waypoint",
        )

        self.publish_f2_goal(self.f2_home_current_goal)

        self.get_logger().warn(
            f"F2 home route started: {self.f2_home_current_goal['name']}"
        )

    def process_home_return_goal_republish(self):
        if not self.home_return_active:
            return

        if self.state != "HOME_RETURNING":
            return

        now = self.now_sec()

        if now - self.home_goal_last_pub_time < self.home_goal_republish_period_sec:
            return

        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "stop",
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "waypoint",
        )

        self.publish_tb3_goal(TB3_HOME_POINT)

        if self.f2_home_current_goal is not None:
            self.publish_f2_goal(self.f2_home_current_goal)

        self.home_goal_last_pub_time = now

    def process_f2_home_route(self):
        if not self.f2_home_route_active:
            return

        if self.f2_home_current_goal is None:
            return

        if not self.f2_pose_ok:
            self.get_logger().warn(
                "Waiting for /f2/relative_pose before checking F2 home waypoint arrival."
            )
            return

        dx = self.f2_home_current_goal["x"] - self.f2_x
        dy = self.f2_home_current_goal["y"] - self.f2_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.f2_home_goal_reached_distance:
            return

        self.get_logger().warn(
            f"F2 reached {self.f2_home_current_goal['name']}: "
            f"dist={dist:.3f}, f2=({self.f2_x:.3f},{self.f2_y:.3f})"
        )

        self.f2_home_route_index += 1

        if self.f2_home_route_index >= len(self.f2_home_route):
            self.f2_home_route_active = False
            self.f2_home_current_goal = None

            self.publish_robot_mode(
                self.f2_mode_pub,
                "f2",
                "stop",
            )

            self.publish_status(
                "WAIT_HOME_DONE",
                "F2 reached HOME route final point. Send home_done after TB3 is also home.",
            )

            return

        self.f2_home_current_goal = self.f2_home_route[self.f2_home_route_index]
        self.f2_home_goal_last_pub_time = 0.0

        self.publish_status(
            "HOME_RETURNING",
            f"F2 moving to {self.f2_home_current_goal['name']}. TB3 is returning home.",
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "waypoint",
        )

        self.publish_f2_goal(self.f2_home_current_goal)

    def handle_home_done(self):
        self.home_return_active = False
        self.f2_home_route_active = False
        self.f2_home_current_goal = None

        self.stop_tb3_route_tracking()
        self.formation_wait_start_time = None
        self.f2_astar_start_time = None

        self.publish_f1_load_exit_cmd("s")
        self.publish_f1_robot_cmd("s")

        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "stop",
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "stop",
        )

        self.publish_status(
            "MISSION_COMPLETE",
            "Home return complete. Mission complete.",
        )

    def stop_tb3_route_tracking(self):
        self.tb3_route_active = False
        self.tb3_route = []
        self.tb3_route_index = 0
        self.tb3_current_goal = None
        self.tb3_goal_last_pub_time = 0.0

    def process_tb3_route(self):
        if not self.tb3_route_active:
            return

        if self.tb3_current_goal is None:
            return

        now = self.now_sec()

        if now - self.tb3_goal_last_pub_time >= self.tb3_goal_republish_period_sec:
            self.publish_tb3_route_goal(force=True)

        if not self.tb3_pose_ok:
            self.get_logger().warn(
                "Waiting for /amcl_pose before checking TB3 goal arrival."
            )
            return

        dx = self.tb3_current_goal["x"] - self.tb3_x
        dy = self.tb3_current_goal["y"] - self.tb3_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.tb3_goal_reached_distance:
            return

        self.get_logger().warn(
            f"TB3 reached {self.tb3_current_goal['name']}: "
            f"dist={dist:.3f}, tb3=({self.tb3_x:.3f},{self.tb3_y:.3f})"
        )

        self.tb3_route_index += 1

        if self.tb3_route_index >= len(self.tb3_route):
            self.stop_tb3_route_tracking()

            self.publish_status(
                "WAIT_LOAD_2",
                "TB3 reached LOAD_POINT_2. F2 should be near approach point. Use follow_159 when 159 marker is visible.",
            )

            return

        self.tb3_current_goal = self.tb3_route[self.tb3_route_index]
        self.tb3_goal_last_pub_time = 0.0

        if self.tb3_current_goal["name"] == "RIGHT_WALL_WAYPOINT_2":
            next_state = "TB3_TO_LOAD2_WP2"
        elif self.tb3_current_goal["name"] == "LOAD_POINT_2":
            next_state = "TB3_TO_LOAD2_FINAL"
        else:
            next_state = "TB3_TO_LOAD2_ROUTE"

        self.publish_status(
            next_state,
            f"TB3 moving to {self.tb3_current_goal['name']}",
        )

        self.publish_tb3_route_goal(force=True)

    def publish_tb3_route_goal(self, force=False):
        if self.tb3_current_goal is None:
            return

        now = self.now_sec()

        if (
            not force
            and now - self.tb3_goal_last_pub_time
            < self.tb3_goal_republish_period_sec
        ):
            return

        self.publish_tb3_goal(self.tb3_current_goal)
        self.tb3_goal_last_pub_time = now

    def stop_mission(self, reason):
        self.home_return_active = False
        self.f2_home_route_active = False
        self.f2_home_current_goal = None

        self.stop_tb3_route_tracking()
        self.formation_wait_start_time = None
        self.f2_astar_start_time = None

        self.publish_f1_load_exit_cmd("s")
        self.publish_f1_robot_cmd("s")

        self.publish_robot_mode(
            self.f1_mode_pub,
            "f1",
            "stop",
        )

        self.publish_robot_mode(
            self.f2_mode_pub,
            "f2",
            "stop",
        )

        self.publish_status(
            "STOPPED",
            f"Mission stopped by {reason}",
        )

    def publish_robot_mode(self, publisher, robot_name, mode):
        msg = String()
        msg.data = mode
        publisher.publish(msg)

        self.get_logger().info(
            f"{robot_name} mode published: {mode}"
        )

    def publish_target_marker(self, publisher, robot_name, marker_id):
        msg = String()
        msg.data = str(marker_id)
        publisher.publish(msg)

        self.get_logger().info(
            f"{robot_name} target_marker_cmd published: {marker_id}"
        )

    def publish_f1_robot_cmd(self, command):
        msg = String()
        msg.data = command
        self.f1_robot_cmd_pub.publish(msg)

        self.get_logger().info(
            f"F1 robot_cmd published: {command}"
        )

    def publish_f1_load_exit_cmd(self, command):
        msg = String()
        msg.data = command
        self.f1_load_exit_cmd_pub.publish(msg)

        self.get_logger().info(
            f"F1 load_exit_cmd published: {command}"
        )

    def publish_tb3_goal(self, point):
        msg = self.make_pose_stamped(point)
        self.tb3_goal_pub.publish(msg)

        self.get_logger().info(
            f"TB3 goal published to /goal_pose: {point['name']} "
            f"x={point['x']:.3f}, y={point['y']:.3f}, yaw={point['yaw']:.3f}"
        )

    def publish_f1_goal(self, point):
        msg = self.make_pose_stamped(point)
        self.f1_goal_pub.publish(msg)

        self.get_logger().info(
            f"F1 goal published to /f1/goal_pose: {point['name']} "
            f"x={point['x']:.3f}, y={point['y']:.3f}, yaw={point['yaw']:.3f}"
        )

    def publish_f2_goal(self, point):
        msg = self.make_pose_stamped(point)
        self.f2_goal_pub.publish(msg)
        self.f2_home_goal_last_pub_time = self.now_sec()

        self.get_logger().warn(
            f"F2 waypoint goal published to /f2/goal_pose: {point['name']} "
            f"x={point['x']:.3f}, y={point['y']:.3f}, yaw={point['yaw']:.3f}"
        )

    def publish_f2_astar_goal(self, point):
        msg = self.make_pose_stamped(point)
        self.f2_astar_goal_pub.publish(msg)
        self.f2_astar_goal_last_pub_time = self.now_sec()

        self.get_logger().warn(
            f"F2 A* goal published to /f2/astar_goal: {point['name']} "
            f"x={point['x']:.3f}, y={point['y']:.3f}, yaw={point['yaw']:.3f}"
        )

    def make_pose_stamped(self, point):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(point["x"])
        msg.pose.position.y = float(point["y"])
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(point["qz"])
        msg.pose.orientation.w = float(point["qw"])

        return msg

    def timer_callback(self):
        self.process_f2_astar_goal_republish()
        self.process_home_return_goal_republish()

        if self.state == "FORMATION_WAIT":
            self.process_formation_wait()

        elif self.state == "F1_LOAD_EXIT_ROTATE":
            self.process_f1_load_exit_rotate()

        elif self.state == "F1_LOAD_EXIT_SETTLE":
            self.process_f1_load_exit_settle()

        elif self.state in [
            "TB3_TO_LOAD2_WP1",
            "TB3_TO_LOAD2_WP2",
            "TB3_TO_LOAD2_FINAL",
            "TB3_TO_LOAD2_ROUTE",
        ]:
            self.process_tb3_route()

        elif self.state == "HOME_RETURNING":
            self.process_f2_home_route()


def main():
    rclpy.init()
    node = MissionManager()

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
