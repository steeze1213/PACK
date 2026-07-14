#!/usr/bin/env python3
import math
import os
import heapq
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class MapAStarPlannerNode(Node):
    def __init__(self):
        super().__init__('f1_map_astar_planner_node')

        self.declare_parameter('map_yaml_path', os.path.expanduser('~/map_cleaned.yaml'))
        self.declare_parameter('path_topic', '/f1/path_points')
        self.declare_parameter('path_vis_topic', '/f1/planned_path')
        self.declare_parameter('relative_pose_topic', '/f1/relative_pose')
        self.declare_parameter('goal_pose_topic', '/f1/goal_pose')
        self.declare_parameter('robot_radius_m', 0.24)
        self.declare_parameter('safety_margin_m', 0.11)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('waypoint_spacing_m', 0.12)
        self.declare_parameter('max_waypoints', 100)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('goal_search_radius_m', 1.80)
        self.declare_parameter('start_search_radius_m', 3.00)
        self.declare_parameter('pose_timeout_sec', 0.80)

        self.map_yaml_path = self.get_parameter('map_yaml_path').value
        self.path_topic = self.get_parameter('path_topic').value
        self.path_vis_topic = self.get_parameter('path_vis_topic').value
        self.relative_pose_topic = self.get_parameter('relative_pose_topic').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.robot_radius_m = float(self.get_parameter('robot_radius_m').value)
        self.safety_margin_m = float(self.get_parameter('safety_margin_m').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.waypoint_spacing_m = float(self.get_parameter('waypoint_spacing_m').value)
        self.max_waypoints = int(self.get_parameter('max_waypoints').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.goal_search_radius_m = float(self.get_parameter('goal_search_radius_m').value)
        self.start_search_radius_m = float(self.get_parameter('start_search_radius_m').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)

        self.map_info = self.load_yaml_simple(self.map_yaml_path)
        self.map_image_path = self.resolve_image_path(self.map_yaml_path, self.map_info.get('image', ''))
        self.resolution = float(self.map_info.get('resolution', 0.05))
        self.origin = self.parse_origin(self.map_info.get('origin', '[-1.78, -5.08, 0]'))
        self.origin_x = float(self.origin[0])
        self.origin_y = float(self.origin[1])

        self.raw_map = self.load_map_image(self.map_image_path)
        self.height, self.width = self.raw_map.shape[:2]

        self.obstacle_grid = self.make_obstacle_grid(self.raw_map)
        self.inflated_grid = self.inflate_obstacles(self.obstacle_grid)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.pose_source = 'none'
        self.pose_quality = 'LOST'
        self.pose_usable = False
        self.last_pose_time = None
        self.last_goal = None

        self.path_pub = self.create_publisher(String, self.path_topic, 10)
        self.path_vis_pub = self.create_publisher(Path, self.path_vis_topic, 10)

        self.pose_sub = self.create_subscription(
            String,
            self.relative_pose_topic,
            self.relative_pose_callback,
            20
        )

        self.goal_sub = self.create_subscription(
            String,
            self.goal_pose_topic,
            self.goal_pose_callback,
            10
        )

        self.get_logger().info('map_astar_planner_node started.')
        self.get_logger().info(f'map_yaml: {self.map_yaml_path}')
        self.get_logger().info(f'map_image: {self.map_image_path}')
        self.get_logger().info(
            f'map size: {self.width}x{self.height}, '
            f'resolution={self.resolution}, '
            f'origin=({self.origin_x},{self.origin_y})'
        )
        self.get_logger().info(
            f'robot_radius={self.robot_radius_m:.2f}m, '
            f'safety_margin={self.safety_margin_m:.2f}m, '
            f'inflation={(self.robot_radius_m + self.safety_margin_m):.2f}m'
        )
        self.get_logger().info(
            f'Subscribing: {self.relative_pose_topic}, {self.goal_pose_topic}'
        )
        self.get_logger().info(
            f'Publishing: {self.path_topic}, {self.path_vis_topic}'
        )

    def load_yaml_simple(self, path):
        data = {}
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or ':' not in line:
                    continue
                key, value = line.split(':', 1)
                data[key.strip()] = value.strip()
        return data

    def resolve_image_path(self, yaml_path, image_value):
        image_value = str(image_value).strip().strip('"').strip("'")
        if os.path.isabs(image_value):
            return image_value
        return os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_value)

    def parse_origin(self, value):
        if isinstance(value, list):
            return value
        s = str(value).strip()
        s = s.strip('[]')
        parts = [p.strip() for p in s.split(',')]
        return [float(p) for p in parts[:3]]

    def load_map_image(self, path):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f'Failed to load map image: {path}')
        return img

    def make_obstacle_grid(self, img):
        occupied = img < 100
        unknown = (img >= 100) & (img < 250)

        if self.unknown_is_obstacle:
            grid = occupied | unknown
        else:
            grid = occupied

        return grid.astype(np.uint8)

    def inflate_obstacles(self, obstacle_grid):
        inflation_m = self.robot_radius_m + self.safety_margin_m
        radius_cells = max(1, int(math.ceil(inflation_m / self.resolution)))

        kernel_size = radius_cells * 2 + 1
        kernel = np.zeros((kernel_size, kernel_size), dtype=np.uint8)
        cy = radius_cells
        cx = radius_cells

        for y in range(kernel_size):
            for x in range(kernel_size):
                if math.hypot(x - cx, y - cy) <= radius_cells:
                    kernel[y, x] = 1

        inflated = cv2.dilate(obstacle_grid, kernel, iterations=1)
        return inflated.astype(np.uint8)

    def relative_pose_callback(self, msg):
        data = msg.data.strip()
        if not data.startswith('RELPOSE'):
            return

        parsed = self.parse_key_value_string(data)
        try:
            self.current_x = float(parsed.get('x'))
            self.current_y = float(parsed.get('y'))
            self.current_yaw = float(parsed.get('yaw', 0.0))
            self.pose_source = str(parsed.get('source', 'LOST')).strip()
            self.pose_quality = str(
                parsed.get('pose_quality', 'LOST')
            ).strip().upper()
            self.pose_usable = int(parsed.get('pose_usable', 0)) == 1
        except Exception:
            return

        self.last_pose_time = self.get_clock().now()

    def goal_pose_callback(self, msg):
        data = msg.data.strip()

        if data == 'GOAL_CLEAR':
            self.last_goal = None
            self.get_logger().info('GOAL_CLEAR received. A* planner goal cleared.')
            return

        if not data.startswith('GOAL'):
            self.get_logger().warn(f'Ignored non-GOAL message on {self.goal_pose_topic}: {data}')
            return

        parsed = self.parse_key_value_string(data)

        try:
            goal_x = float(parsed.get('x'))
            goal_y = float(parsed.get('y'))
        except Exception:
            self.get_logger().warn(f'Failed to parse GOAL message: {data}')
            return

        self.last_goal = (goal_x, goal_y)

        if not self.is_goal_pose_usable():
            self.publish_failed(
                'pose_not_usable',
                self.current_x if self.current_x is not None else 0.0,
                self.current_y if self.current_y is not None else 0.0,
                goal_x,
                goal_y
            )
            self.get_logger().warn(
                'Goal rejected: fresh GOOD/OK pose_usable=1 is required. '
                f'source={self.pose_source}, quality={self.pose_quality}, '
                f'usable={1 if self.pose_usable else 0}'
            )
            return

        self.plan_and_publish(self.current_x, self.current_y, goal_x, goal_y)

    def is_goal_pose_usable(self):
        if (
            self.current_x is None
            or self.current_y is None
            or self.last_pose_time is None
        ):
            return False

        age = (
            self.get_clock().now() - self.last_pose_time
        ).nanoseconds / 1e9
        return (
            age <= self.pose_timeout_sec
            and self.pose_source.upper() != 'LOST'
            and self.pose_quality in ('GOOD', 'OK')
            and self.pose_usable
        )

    def parse_key_value_string(self, text):
        result = {}
        parts = text.split(',')
        for part in parts:
            part = part.strip()
            if '=' not in part:
                continue
            k, v = part.split('=', 1)
            result[k.strip()] = v.strip()
        return result

    def world_to_grid(self, x, y):
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def grid_to_img(self, gx, gy):
        row = self.height - 1 - gy
        col = gx
        return row, col

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx, gy):
        if not self.in_bounds(gx, gy):
            return False
        row, col = self.grid_to_img(gx, gy)
        return self.inflated_grid[row, col] == 0

    def nearest_free_cell(self, start, radius_m):
        sx, sy = start
        if self.is_free(sx, sy):
            return start

        max_r = int(math.ceil(radius_m / self.resolution))
        visited = set()
        q = deque()
        q.append((sx, sy, 0))
        visited.add((sx, sy))

        while q:
            x, y, d = q.popleft()
            if d > max_r:
                break

            if self.is_free(x, y):
                return x, y

            for nx, ny in self.neighbor_cells_4(x, y):
                if (nx, ny) in visited:
                    continue
                visited.add((nx, ny))
                q.append((nx, ny, d + 1))

        return None

    def neighbor_cells_4(self, x, y):
        return [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]

    def neighbors(self, x, y):
        base = [
            (x + 1, y, 1.0),
            (x - 1, y, 1.0),
            (x, y + 1, 1.0),
            (x, y - 1, 1.0),
        ]

        if not self.allow_diagonal:
            return base

        diag_cost = math.sqrt(2.0)
        diag = [
            (x + 1, y + 1, diag_cost),
            (x + 1, y - 1, diag_cost),
            (x - 1, y + 1, diag_cost),
            (x - 1, y - 1, diag_cost),
        ]
        return base + diag

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def astar(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            cx, cy = current

            for nx, ny, move_cost in self.neighbors(cx, cy):
                if not self.is_free(nx, ny):
                    continue

                if self.allow_diagonal and nx != cx and ny != cy:
                    if not self.is_free(nx, cy) or not self.is_free(cx, ny):
                        continue

                tentative_g = g_score[current] + move_cost

                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + self.heuristic((nx, ny), goal)
                    heapq.heappush(open_heap, (f, (nx, ny)))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def line_is_free(self, a, b):
        for cell in self.bresenham(a[0], a[1], b[0], b[1]):
            if not self.is_free(cell[0], cell[1]):
                return False
        return True

    def bresenham(self, x0, y0, x1, y1):
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x = x0
        y = y0

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        cells.append((x1, y1))
        return cells

    def simplify_line_of_sight(self, path):
        if not path or len(path) <= 2:
            return path

        simplified = [path[0]]
        anchor_idx = 0

        for i in range(2, len(path)):
            if not self.line_is_free(path[anchor_idx], path[i]):
                simplified.append(path[i - 1])
                anchor_idx = i - 1

        simplified.append(path[-1])
        return simplified

    def densify_waypoints(self, world_points):
        if not world_points:
            return []

        out = [world_points[0]]

        for i in range(1, len(world_points)):
            x0, y0 = out[-1]
            x1, y1 = world_points[i]
            dist = math.hypot(x1 - x0, y1 - y0)

            if dist <= self.waypoint_spacing_m:
                out.append((x1, y1))
                continue

            steps = max(1, int(math.ceil(dist / self.waypoint_spacing_m)))
            for s in range(1, steps + 1):
                t = s / steps
                x = x0 + (x1 - x0) * t
                y = y0 + (y1 - y0) * t
                out.append((x, y))

        return out

    def thin_waypoints(self, points):
        if len(points) <= self.max_waypoints:
            return points

        result = []
        n = len(points)

        for i in range(self.max_waypoints):
            idx = int(round(i * (n - 1) / (self.max_waypoints - 1)))
            result.append(points[idx])

        return result

    def plan_and_publish(self, start_x, start_y, goal_x, goal_y):
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)

        if not self.in_bounds(*start_grid):
            self.publish_failed('start_out_of_map', start_x, start_y, goal_x, goal_y)
            return

        if not self.in_bounds(*goal_grid):
            self.publish_failed('goal_out_of_map', start_x, start_y, goal_x, goal_y)
            return

        fixed_start = self.nearest_free_cell(start_grid, self.start_search_radius_m)
        fixed_goal = self.nearest_free_cell(goal_grid, self.goal_search_radius_m)

        if fixed_start is None:
            self.publish_failed('no_free_start_nearby', start_x, start_y, goal_x, goal_y)
            return

        if fixed_goal is None:
            self.publish_failed('no_free_goal_nearby', start_x, start_y, goal_x, goal_y)
            return

        if fixed_start != start_grid:
            sx, sy = self.grid_to_world(*fixed_start)
            self.get_logger().warn(
                f'Start is occupied/inflated. Using nearest free start: '
                f'({start_x:.3f},{start_y:.3f}) -> ({sx:.3f},{sy:.3f})'
            )

        if fixed_goal != goal_grid:
            gx, gy = self.grid_to_world(*fixed_goal)
            self.get_logger().warn(
                f'Goal is occupied/inflated. Using nearest free goal: '
                f'({goal_x:.3f},{goal_y:.3f}) -> ({gx:.3f},{gy:.3f})'
            )

        grid_path = self.astar(fixed_start, fixed_goal)

        if not grid_path:
            self.publish_failed('astar_failed', start_x, start_y, goal_x, goal_y)
            return

        simple_grid_path = self.simplify_line_of_sight(grid_path)
        world_simple = [self.grid_to_world(x, y) for x, y in simple_grid_path]
        world_dense = self.densify_waypoints(world_simple)
        world_dense = self.thin_waypoints(world_dense)

        if world_dense:
            world_dense[0] = (start_x, start_y)
            world_dense[-1] = (goal_x, goal_y)

        self.publish_path(world_dense, start_x, start_y, goal_x, goal_y, len(grid_path))

    def publish_failed(self, reason, start_x, start_y, goal_x, goal_y):
        msg = String()
        msg.data = (
            f'PATH,status=failed,reason={reason},count=0,'
            f'start_x={start_x:.3f},start_y={start_y:.3f},'
            f'goal_x={goal_x:.3f},goal_y={goal_y:.3f}'
        )
        self.path_pub.publish(msg)
        self.get_logger().warn(msg.data)

    def publish_path(self, points, start_x, start_y, goal_x, goal_y, raw_grid_count):
        msg = String()
        parts = [
            'PATH',
            'status=ok',
            f'count={len(points)}',
            f'start_x={start_x:.3f}',
            f'start_y={start_y:.3f}',
            f'goal_x={goal_x:.3f}',
            f'goal_y={goal_y:.3f}',
            f'raw_grid_count={raw_grid_count}',
        ]

        for i, (x, y) in enumerate(points):
            parts.append(f'p{i}_x={x:.3f}')
            parts.append(f'p{i}_y={y:.3f}')

        msg.data = ','.join(parts)
        self.path_pub.publish(msg)

        vis = Path()
        vis.header.stamp = self.get_clock().now().to_msg()
        vis.header.frame_id = 'map'

        for x, y in points:
            ps = PoseStamped()
            ps.header = vis.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            vis.poses.append(ps)

        self.path_vis_pub.publish(vis)

        self.get_logger().info(
            f'A* path published: waypoints={len(points)}, '
            f'raw_grid={raw_grid_count}, '
            f'start=({start_x:.3f},{start_y:.3f}), '
            f'goal=({goal_x:.3f},{goal_y:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapAStarPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
