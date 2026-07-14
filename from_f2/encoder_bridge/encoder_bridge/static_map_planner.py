import heapq
import math
import os
from collections import deque

import cv2
import numpy as np


class StaticMapPlanner:
    def __init__(
        self,
        map_yaml_path,
        robot_radius_m=0.24,
        safety_margin_m=0.11,
        unknown_is_obstacle=True,
        waypoint_spacing_m=0.12,
        allow_diagonal=True,
        goal_search_radius_m=1.80,
    ):
        self.map_yaml_path = os.path.expanduser(map_yaml_path)
        self.robot_radius_m = float(robot_radius_m)
        self.safety_margin_m = float(safety_margin_m)
        self.unknown_is_obstacle = bool(unknown_is_obstacle)
        self.waypoint_spacing_m = float(waypoint_spacing_m)
        self.allow_diagonal = bool(allow_diagonal)
        self.goal_search_radius_m = float(goal_search_radius_m)

        info = self.load_yaml_simple(self.map_yaml_path)
        image_path = self.resolve_image_path(
            self.map_yaml_path, info.get('image', '')
        )
        self.resolution = float(info.get('resolution', 0.05))
        origin = self.parse_origin(info.get('origin', '[-1.78, -5.08, 0]'))
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])

        raw_map = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if raw_map is None:
            raise RuntimeError(f'Failed to load map image: {image_path}')

        self.height, self.width = raw_map.shape[:2]
        obstacle_grid = self.make_obstacle_grid(raw_map)
        self.inflated_grid = self.inflate_obstacles(obstacle_grid)

    @staticmethod
    def load_yaml_simple(path):
        data = {}
        with open(path, 'r', encoding='utf-8') as stream:
            for line in stream:
                line = line.strip()
                if not line or line.startswith('#') or ':' not in line:
                    continue
                key, value = line.split(':', 1)
                data[key.strip()] = value.strip()
        return data

    @staticmethod
    def resolve_image_path(yaml_path, image_value):
        image_value = str(image_value).strip().strip('"').strip("'")
        if os.path.isabs(image_value):
            return image_value
        return os.path.join(
            os.path.dirname(os.path.abspath(yaml_path)), image_value
        )

    @staticmethod
    def parse_origin(value):
        if isinstance(value, list):
            return value
        parts = str(value).strip().strip('[]').split(',')
        return [float(part.strip()) for part in parts[:3]]

    def make_obstacle_grid(self, image):
        occupied = image < 100
        unknown = (image >= 100) & (image < 250)
        grid = occupied | unknown if self.unknown_is_obstacle else occupied
        return grid.astype(np.uint8)

    def inflate_obstacles(self, obstacle_grid):
        inflation_m = self.robot_radius_m + self.safety_margin_m
        radius = max(1, int(math.ceil(inflation_m / self.resolution)))
        size = radius * 2 + 1
        kernel = np.zeros((size, size), dtype=np.uint8)
        for y in range(size):
            for x in range(size):
                if math.hypot(x - radius, y - radius) <= radius:
                    kernel[y, x] = 1
        return cv2.dilate(obstacle_grid, kernel, iterations=1).astype(np.uint8)

    def world_to_grid(self, x, y):
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        return (
            self.origin_x + (gx + 0.5) * self.resolution,
            self.origin_y + (gy + 0.5) * self.resolution,
        )

    def grid_to_img(self, gx, gy):
        return self.height - 1 - gy, gx

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx, gy):
        if not self.in_bounds(gx, gy):
            return False
        row, col = self.grid_to_img(gx, gy)
        return self.inflated_grid[row, col] == 0

    @staticmethod
    def neighbor_cells_4(x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def nearest_free_cell(self, start, radius_m):
        if self.is_free(*start):
            return start
        max_radius = int(math.ceil(radius_m / self.resolution))
        queue = deque([(start[0], start[1], 0)])
        visited = {start}
        while queue:
            x, y, distance = queue.popleft()
            if distance > max_radius:
                break
            if self.is_free(x, y):
                return x, y
            for neighbor in self.neighbor_cells_4(x, y):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                queue.append((neighbor[0], neighbor[1], distance + 1))
        return None

    def neighbors(self, x, y):
        result = [
            (x + 1, y, 1.0),
            (x - 1, y, 1.0),
            (x, y + 1, 1.0),
            (x, y - 1, 1.0),
        ]
        if self.allow_diagonal:
            diagonal = math.sqrt(2.0)
            result.extend([
                (x + 1, y + 1, diagonal),
                (x + 1, y - 1, diagonal),
                (x - 1, y + 1, diagonal),
                (x - 1, y - 1, diagonal),
            ])
        return result

    @staticmethod
    def heuristic(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def astar(self, start, goal):
        open_heap = [(0.0, start)]
        came_from = {}
        score = {start: 0.0}
        closed = set()
        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            cx, cy = current
            for nx, ny, cost in self.neighbors(cx, cy):
                if not self.is_free(nx, ny):
                    continue
                if self.allow_diagonal and nx != cx and ny != cy:
                    if not self.is_free(nx, cy) or not self.is_free(cx, ny):
                        continue
                tentative = score[current] + cost
                if tentative >= score.get((nx, ny), float('inf')):
                    continue
                came_from[(nx, ny)] = current
                score[(nx, ny)] = tentative
                total = tentative + self.heuristic((nx, ny), goal)
                heapq.heappush(open_heap, (total, (nx, ny)))
        return None

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dx > dy:
            error = dx / 2.0
            while x != x1:
                cells.append((x, y))
                error -= dy
                if error < 0:
                    y += sy
                    error += dx
                x += sx
        else:
            error = dy / 2.0
            while y != y1:
                cells.append((x, y))
                error -= dx
                if error < 0:
                    x += sx
                    error += dy
                y += sy
        cells.append((x1, y1))
        return cells

    def line_is_free(self, start, goal):
        return all(self.is_free(*cell) for cell in self.bresenham(
            start[0], start[1], goal[0], goal[1]
        ))

    def simplify_line_of_sight(self, path):
        if not path or len(path) <= 2:
            return path
        simplified = [path[0]]
        anchor = 0
        for index in range(2, len(path)):
            if not self.line_is_free(path[anchor], path[index]):
                simplified.append(path[index - 1])
                anchor = index - 1
        simplified.append(path[-1])
        return simplified

    def densify(self, points):
        if not points:
            return []
        result = [points[0]]
        for x1, y1 in points[1:]:
            x0, y0 = result[-1]
            distance = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(math.ceil(
                distance / self.waypoint_spacing_m
            )))
            for step in range(1, steps + 1):
                ratio = step / steps
                result.append((
                    x0 + (x1 - x0) * ratio,
                    y0 + (y1 - y0) * ratio,
                ))
        return result

    def plan_route(self, start_x, start_y, goal_x, goal_y):
        start = self.world_to_grid(start_x, start_y)
        requested_goal = self.world_to_grid(goal_x, goal_y)
        result = {
            'status': 'failed',
            'reason': '',
            'line_free': False,
            'path': [],
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_adjusted': False,
        }
        if not self.in_bounds(*start):
            result['reason'] = 'start_out_of_map'
            return result
        if not self.is_free(*start):
            result['reason'] = 'start_in_inflated_obstacle'
            return result
        if not self.in_bounds(*requested_goal):
            result['reason'] = 'goal_out_of_map'
            return result

        goal = self.nearest_free_cell(
            requested_goal, self.goal_search_radius_m
        )
        if goal is None:
            result['reason'] = 'no_free_goal_nearby'
            return result
        if goal != requested_goal:
            result['goal_adjusted'] = True
            result['goal_x'], result['goal_y'] = self.grid_to_world(*goal)

        if goal == requested_goal and self.line_is_free(start, goal):
            result.update({
                'status': 'direct',
                'reason': 'line_free',
                'line_free': True,
                'path': [(start_x, start_y), (goal_x, goal_y)],
            })
            return result

        path = self.astar(start, goal)
        if not path:
            result['reason'] = 'astar_failed'
            return result
        simple = self.simplify_line_of_sight(path)
        world = [self.grid_to_world(*cell) for cell in simple]
        result.update({
            'status': 'astar',
            'reason': (
                'goal_adjusted' if result['goal_adjusted']
                else 'line_blocked'
            ),
            'path': self.densify(world),
        })
        return result
