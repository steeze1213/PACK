#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


LOAD_POINT_1 = {
    "name": "LOAD_POINT_1",
    "x": 2.068,
    "y": -0.070,
    "yaw": 0.035,
    "qz": 0.017484554994104,
    "qw": 0.9998471334842433,
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


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.state = "IDLE"

        self.mission_cmd_sub = self.create_subscription(
            String,
            "/mission_cmd",
            self.mission_cmd_callback,
            10,
        )

        # TB3 Nav2는 /goal_pose를 구독한다.
        # /tb3/goal_pose가 아니라 /goal_pose로 보내야 bt_navigator가 받는다.
        self.tb3_goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10,
        )

        self.mission_status_pub = self.create_publisher(
            String,
            "/mission_status",
            10,
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("mission_manager_node started")
        self.publish_status("IDLE", "Mission manager ready")

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
        elif cmd in ["pause", "stop", "emergency_stop"]:
            self.stop_mission(cmd)
        else:
            self.publish_status("ERROR", f"Unknown mission_cmd: {cmd}")

    def start_mission(self):
        self.publish_status("GO_TO_LOAD_1", "TB3 moving to LOAD_POINT_1")
        self.publish_tb3_goal(LOAD_POINT_1)

    def stop_mission(self, reason):
        self.publish_status("STOPPED", f"Mission stopped by {reason}")

    def publish_tb3_goal(self, point):
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

        self.tb3_goal_pub.publish(msg)

        self.get_logger().info(
            f"TB3 goal published to /goal_pose: {point['name']} "
            f"x={point['x']:.3f}, y={point['y']:.3f}, yaw={point['yaw']:.3f}"
        )

    def timer_callback(self):
        pass


def main():
    rclpy.init()
    node = MissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
