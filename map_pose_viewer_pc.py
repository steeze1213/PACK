import math
import os

import cv2
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MapPoseViewer(Node):
    def __init__(self):
        super().__init__('map_pose_viewer_pc')

        self.map_image_path = os.path.expanduser('~/map_cleaned.pgm')
        self.map_yaml_path = os.path.expanduser('~/map_cleaned.yaml')
        self.aruco_yaml_path = os.path.expanduser('~/aruco_reference.yaml')

        self.map_img = cv2.imread(
            self.map_image_path,
            cv2.IMREAD_GRAYSCALE
        )

        if self.map_img is None:
            raise RuntimeError(
                f'Map image open failed: {self.map_image_path}'
            )

        map_info = self.load_yaml_safely(
            self.map_yaml_path,
            'Map YAML'
        )

        if map_info is None:
            raise RuntimeError(
                f'Map YAML open failed: {self.map_yaml_path}'
            )

        self.resolution = float(map_info['resolution'])
        self.origin_x = float(map_info['origin'][0])
        self.origin_y = float(map_info['origin'][1])

        self.map_h, self.map_w = self.map_img.shape[:2]
        self.scale = 6

        self.latest_pose = None
        self.latest_goal = None

        self.aruco_markers = self.load_aruco_markers()

        self.pose_trail = []
        self.max_trail_len = 300

        self.last_pose_for_jump = None
        self.jump_warning = None
        self.jump_warning_count = 0

        self.show_markers = True
        self.show_trail = True
        self.show_marker_id_text = True

        self.jump_dist_threshold = 0.25
        self.jump_yaw_threshold_deg = 25.0

        self.pose_sub = self.create_subscription(
            String,
            '/relative_pose',
            self.pose_callback,
            10
        )

        self.goal_pub = self.create_publisher(
            String,
            '/goal_pose',
            10
        )

        self.window_name = 'Relative Pose on Map'

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(
            self.window_name,
            self.mouse_callback
        )

        self.timer = self.create_timer(
            0.05,
            self.draw_loop
        )

        self.get_logger().info('map_pose_viewer_pc started.')
        self.get_logger().info(f'map image: {self.map_image_path}')
        self.get_logger().info(f'map yaml: {self.map_yaml_path}')
        self.get_logger().info(f'aruco yaml: {self.aruco_yaml_path}')
        self.get_logger().info(f'resolution: {self.resolution}')
        self.get_logger().info(
            f'origin: x={self.origin_x}, y={self.origin_y}'
        )
        self.get_logger().info(
            f'loaded markers: {len(self.aruco_markers)}'
        )
        self.get_logger().info('Marker range: 10~68')
        self.get_logger().info('Subscribing: /relative_pose')
        self.get_logger().info('Publishing: /goal_pose')
        self.get_logger().info('Left click: publish goal')
        self.get_logger().info('Press c: clear goal display')
        self.get_logger().info('Press s: publish GOAL_CLEAR')
        self.get_logger().info('Press m: show/hide ArUco markers')
        self.get_logger().info('Press t: show/hide robot trail')
        self.get_logger().info('Press i: show/hide marker ID')
        self.get_logger().info('Press r: reset trail and jump checker')
        self.get_logger().info('Press q: quit')

    def load_yaml_safely(self, path, label):
        if not os.path.exists(path):
            self.get_logger().warn(
                f'{label} not found: {path}'
            )
            return None

        try:
            with open(path, 'r', encoding='utf-8') as file:
                raw_text = file.read()
        except Exception as error:
            self.get_logger().warn(
                f'{label} read failed: {error}'
            )
            return None

        cleaned_lines = []
        removed_count = 0

        for line in raw_text.splitlines():
            stripped = line.strip()

            if stripped in (
                '```',
                '```yaml',
                '```yml',
                '```python'
            ):
                removed_count += 1
                continue

            cleaned_lines.append(line)

        cleaned_text = '\n'.join(cleaned_lines)

        if removed_count > 0:
            self.get_logger().warn(
                f'{label}: removed {removed_count} code fence line(s).'
            )

        try:
            return yaml.safe_load(cleaned_text)
        except yaml.YAMLError as error:
            self.get_logger().warn(
                f'{label} YAML parse failed: {error}'
            )
            return None
        except Exception as error:
            self.get_logger().warn(
                f'{label} load failed: {error}'
            )
            return None

    def load_aruco_markers(self):
        data = self.load_yaml_safely(
            self.aruco_yaml_path,
            'ArUco YAML'
        )

        if data is None:
            return {}

        if not isinstance(data, dict):
            self.get_logger().warn(
                'ArUco YAML root must be a dictionary.'
            )
            return {}

        marker_data = data.get('aruco_marker_pose', {})

        if not isinstance(marker_data, dict):
            self.get_logger().warn(
                'aruco_marker_pose must be a dictionary.'
            )
            return {}

        markers = {}

        for marker_id, pose in marker_data.items():
            try:
                mid = int(marker_id)

                if not 10 <= mid <= 68:
                    continue

                if not isinstance(pose, dict):
                    raise ValueError(
                        'marker pose is not a dictionary'
                    )

                markers[mid] = {
                    'x': float(pose.get('x', 0.0)),
                    'y': float(pose.get('y', 0.0)),
                    'z': float(pose.get('z', 0.0)),
                    'yaw': float(pose.get('yaw', 0.0))
                }

            except (
                TypeError,
                ValueError,
                AttributeError
            ) as error:
                self.get_logger().warn(
                    f'Invalid marker data: '
                    f'marker_id={marker_id}, error={error}'
                )

        self.get_logger().info(
            f'Loaded ArUco markers 10~68: {len(markers)}'
        )

        if len(markers) == 0:
            self.get_logger().warn(
                'No ArUco markers loaded. '
                'Check aruco_reference.yaml.'
            )

        return markers

    def pose_callback(self, msg):
        parsed = self.parse_relpose(
            msg.data.strip()
        )

        if parsed is None:
            return

        self.check_pose_jump(parsed)
        self.latest_pose = parsed

        try:
            x = float(parsed.get('x', 0.0))
            y = float(parsed.get('y', 0.0))
            yaw = float(parsed.get('yaw', 0.0))

            self.pose_trail.append(
                (x, y, yaw)
            )

            if len(self.pose_trail) > self.max_trail_len:
                self.pose_trail.pop(0)

        except (TypeError, ValueError):
            pass

    def check_pose_jump(self, pose):
        try:
            x = float(pose.get('x', 0.0))
            y = float(pose.get('y', 0.0))
            yaw = float(pose.get('yaw', 0.0))
            marker_id = pose.get('marker_id', 'none')
        except (TypeError, ValueError):
            return

        if self.last_pose_for_jump is None:
            self.last_pose_for_jump = {
                'x': x,
                'y': y,
                'yaw': yaw,
                'marker_id': marker_id
            }
            return

        last_x = self.last_pose_for_jump['x']
        last_y = self.last_pose_for_jump['y']
        last_yaw = self.last_pose_for_jump['yaw']
        last_marker_id = self.last_pose_for_jump['marker_id']

        dist_jump = math.hypot(
            x - last_x,
            y - last_y
        )

        yaw_jump = abs(
            self.normalize_angle(
                yaw - last_yaw
            )
        )

        yaw_jump_deg = math.degrees(
            yaw_jump
        )

        if (
            dist_jump > self.jump_dist_threshold
            or yaw_jump_deg > self.jump_yaw_threshold_deg
        ):
            self.jump_warning = (
                f'POSE JUMP! marker '
                f'{last_marker_id} -> {marker_id}, '
                f'dist={dist_jump:.3f}m, '
                f'yaw={yaw_jump_deg:.1f}deg'
            )

            self.jump_warning_count = 60

            self.get_logger().warn(
                self.jump_warning
            )

        self.last_pose_for_jump = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'marker_id': marker_id
        }

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def parse_relpose(self, data):
        if not data.startswith('RELPOSE,'):
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

    def map_to_pixel(self, x, y):
        px = int(
            (x - self.origin_x)
            / self.resolution
        )

        py = int(
            self.map_h
            - (
                (y - self.origin_y)
                / self.resolution
            )
        )

        return px, py

    def pixel_to_map(self, px, py):
        map_px = px / self.scale
        map_py = py / self.scale

        x = (
            map_px * self.resolution
            + self.origin_x
        )

        y = (
            (self.map_h - map_py)
            * self.resolution
            + self.origin_y
        )

        return x, y

    def mouse_callback(
        self,
        event,
        x,
        y,
        flags,
        param
    ):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        goal_x, goal_y = self.pixel_to_map(
            x,
            y
        )

        self.latest_goal = {
            'x': goal_x,
            'y': goal_y
        }

        msg = String()
        msg.data = (
            f'GOAL,x={goal_x:.3f},'
            f'y={goal_y:.3f}'
        )

        self.goal_pub.publish(msg)

        self.get_logger().info(
            f'Published goal: '
            f'x={goal_x:.3f}, '
            f'y={goal_y:.3f}'
        )

    def publish_goal_clear(self):
        msg = String()
        msg.data = 'GOAL_CLEAR'

        self.goal_pub.publish(msg)
        self.latest_goal = None

        self.get_logger().info(
            'Published GOAL_CLEAR'
        )

    def get_marker_style(
        self,
        marker_id,
        current_marker_id
    ):
        try:
            is_current = (
                int(current_marker_id)
                == int(marker_id)
            )
        except (TypeError, ValueError):
            is_current = False

        if is_current:
            return (255, 0, 255), 8, -1

        if 10 <= marker_id <= 20:
            return (0, 255, 255), 5, -1

        if 21 <= marker_id <= 50:
            return (255, 180, 0), 5, -1

        if 51 <= marker_id <= 58:
            return (0, 165, 255), 5, -1

        if 59 <= marker_id <= 68:
            return (255, 100, 255), 5, -1

        return (180, 180, 180), 4, -1

    def draw_aruco_markers(
        self,
        display,
        current_marker_id
    ):
        for marker_id in sorted(
            self.aruco_markers.keys()
        ):
            marker = self.aruco_markers[
                marker_id
            ]

            x = float(marker['x'])
            y = float(marker['y'])
            yaw = float(marker['yaw'])

            px, py = self.map_to_pixel(
                x,
                y
            )

            sx = px * self.scale
            sy = py * self.scale

            if sx < 0 or sy < 0:
                continue

            if (
                sx >= display.shape[1]
                or sy >= display.shape[0]
            ):
                continue

            marker_color, radius, thickness = (
                self.get_marker_style(
                    marker_id,
                    current_marker_id
                )
            )

            cv2.circle(
                display,
                (sx, sy),
                radius,
                marker_color,
                thickness
            )

            cv2.circle(
                display,
                (sx, sy),
                radius + 3,
                (0, 0, 0),
                1
            )

            arrow_len = 30

            ex = int(
                sx
                + arrow_len
                * math.cos(yaw)
            )

            ey = int(
                sy
                - arrow_len
                * math.sin(yaw)
            )

            cv2.arrowedLine(
                display,
                (sx, sy),
                (ex, ey),
                marker_color,
                2,
                tipLength=0.35
            )

            if self.show_marker_id_text:
                cv2.putText(
                    display,
                    str(marker_id),
                    (sx + 7, sy - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    marker_color,
                    2
                )

    def draw_trail(self, display):
        if len(self.pose_trail) < 2:
            return

        points = []

        for x, y, yaw in self.pose_trail:
            px, py = self.map_to_pixel(
                x,
                y
            )

            sx = px * self.scale
            sy = py * self.scale

            points.append(
                (sx, sy)
            )

        for index in range(
            1,
            len(points)
        ):
            cv2.line(
                display,
                points[index - 1],
                points[index],
                (255, 0, 0),
                2
            )

    def draw_legend(self, display):
        x0 = 20
        y0 = display.shape[0] - 190

        legend_lines = [
            ('Robot', (0, 0, 255)),
            ('Marker 10~20', (0, 255, 255)),
            ('Marker 21~50', (255, 180, 0)),
            ('Marker 51~58', (0, 165, 255)),
            ('Marker 59~68', (255, 100, 255)),
            ('Current marker', (255, 0, 255)),
            ('Trail', (255, 0, 0)),
            ('Goal', (0, 255, 0))
        ]

        box_height = (
            len(legend_lines) * 22
            + 10
        )

        cv2.rectangle(
            display,
            (x0 - 10, y0 - 25),
            (
                x0 + 260,
                y0 + box_height
            ),
            (0, 0, 0),
            -1
        )

        for index, item in enumerate(
            legend_lines
        ):
            name, color = item
            yy = y0 + index * 22

            cv2.circle(
                display,
                (x0, yy),
                5,
                color,
                -1
            )

            cv2.putText(
                display,
                name,
                (x0 + 15, yy + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1
            )

    def draw_loop(self):
        display = cv2.cvtColor(
            self.map_img,
            cv2.COLOR_GRAY2BGR
        )

        display = cv2.resize(
            display,
            (
                self.map_w * self.scale,
                self.map_h * self.scale
            ),
            interpolation=cv2.INTER_NEAREST
        )

        current_marker_id = 'none'

        if self.latest_pose is not None:
            current_marker_id = self.latest_pose.get(
                'marker_id',
                'none'
            )

        if self.show_markers:
            self.draw_aruco_markers(
                display,
                current_marker_id
            )

        if self.show_trail:
            self.draw_trail(display)

        if self.latest_goal is not None:
            gx = float(
                self.latest_goal['x']
            )

            gy = float(
                self.latest_goal['y']
            )

            gpx, gpy = self.map_to_pixel(
                gx,
                gy
            )

            gsx = gpx * self.scale
            gsy = gpy * self.scale

            cv2.circle(
                display,
                (gsx, gsy),
                9,
                (0, 255, 0),
                -1
            )

            cv2.circle(
                display,
                (gsx, gsy),
                14,
                (0, 0, 0),
                2
            )

            cv2.putText(
                display,
                f'GOAL ({gx:.2f}, {gy:.2f})',
                (gsx + 12, gsy - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 180, 0),
                2
            )

        if self.latest_pose is not None:
            x = float(
                self.latest_pose.get(
                    'x',
                    0.0
                )
            )

            y = float(
                self.latest_pose.get(
                    'y',
                    0.0
                )
            )

            yaw = float(
                self.latest_pose.get(
                    'yaw',
                    0.0
                )
            )

            source = self.latest_pose.get(
                'source',
                'unknown'
            )

            rel_x = float(
                self.latest_pose.get(
                    'rel_x',
                    0.0
                )
            )

            rel_z = float(
                self.latest_pose.get(
                    'rel_z',
                    0.0
                )
            )

            marker_seen = int(
                self.latest_pose.get(
                    'marker_seen',
                    0
                )
            )

            marker_id = self.latest_pose.get(
                'marker_id',
                'none'
            )

            cmd = self.latest_pose.get(
                'cmd',
                'none'
            )

            px, py = self.map_to_pixel(
                x,
                y
            )

            sx = px * self.scale
            sy = py * self.scale

            cv2.circle(
                display,
                (sx, sy),
                8,
                (0, 0, 255),
                -1
            )

            cv2.circle(
                display,
                (sx, sy),
                12,
                (0, 0, 0),
                2
            )

            arrow_len = 45

            ex = int(
                sx
                + arrow_len
                * math.cos(yaw)
            )

            ey = int(
                sy
                - arrow_len
                * math.sin(yaw)
            )

            cv2.arrowedLine(
                display,
                (sx, sy),
                (ex, ey),
                (0, 0, 255),
                3,
                tipLength=0.35
            )

            text_lines = [
                (
                    f'ROBOT x={x:.3f}, '
                    f'y={y:.3f}, '
                    f'yaw={math.degrees(yaw):.1f}deg'
                ),
                (
                    f'source={source}, '
                    f'marker_id={marker_id}, '
                    f'marker_seen={marker_seen}, '
                    f'cmd={cmd}'
                ),
                (
                    f'rel_x={rel_x:.3f}, '
                    f'rel_z={rel_z:.3f}'
                ),
                f'pixel=({px}, {py})',
                (
                    f'markers_loaded='
                    f'{len(self.aruco_markers)}, '
                    f'show_markers='
                    f'{self.show_markers}, '
                    f'show_trail='
                    f'{self.show_trail}'
                ),
                (
                    'Left click: goal / '
                    'c: clear display / '
                    's: stop / '
                    'm: markers / '
                    't: trail / '
                    'i: IDs / '
                    'r: reset / '
                    'q: quit'
                )
            ]

            if self.latest_goal is not None:
                gx = float(
                    self.latest_goal['x']
                )

                gy = float(
                    self.latest_goal['y']
                )

                dist = math.hypot(
                    gx - x,
                    gy - y
                )

                text_lines.append(
                    f'goal=({gx:.3f}, '
                    f'{gy:.3f}), '
                    f'dist={dist:.3f}m'
                )

            for index, line in enumerate(
                text_lines
            ):
                cv2.putText(
                    display,
                    line,
                    (
                        20,
                        25 + index * 28
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 0, 255),
                    2
                )

        else:
            cv2.putText(
                display,
                'Waiting for /relative_pose...',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2
            )

        self.draw_legend(display)

        if (
            self.jump_warning_count > 0
            and self.jump_warning is not None
        ):
            cv2.rectangle(
                display,
                (
                    15,
                    display.shape[0] - 70
                ),
                (
                    display.shape[1] - 15,
                    display.shape[0] - 20
                ),
                (0, 0, 0),
                -1
            )

            cv2.putText(
                display,
                self.jump_warning,
                (
                    25,
                    display.shape[0] - 38
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (0, 0, 255),
                2
            )

            self.jump_warning_count -= 1

        cv2.imshow(
            self.window_name,
            display
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            rclpy.shutdown()

        elif key == ord('c'):
            self.latest_goal = None

            self.get_logger().info(
                'Cleared goal display only.'
            )

        elif key == ord('s'):
            self.publish_goal_clear()

        elif key == ord('m'):
            self.show_markers = not self.show_markers

            self.get_logger().info(
                f'show_markers={self.show_markers}'
            )

        elif key == ord('t'):
            self.show_trail = not self.show_trail

            self.get_logger().info(
                f'show_trail={self.show_trail}'
            )

        elif key == ord('i'):
            self.show_marker_id_text = (
                not self.show_marker_id_text
            )

            self.get_logger().info(
                f'show_marker_id_text='
                f'{self.show_marker_id_text}'
            )

        elif key == ord('r'):
            self.pose_trail = []
            self.last_pose_for_jump = None
            self.jump_warning = None
            self.jump_warning_count = 0

            self.get_logger().info(
                'Reset trail and jump checker.'
            )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MapPoseViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
