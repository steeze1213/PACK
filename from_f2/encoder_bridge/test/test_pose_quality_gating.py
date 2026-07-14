from encoder_bridge.relative_pose_node import RelativePoseNode
from encoder_bridge.turtlebot_to_f1_follow_node import TurtlebotToF1FollowNode
from encoder_bridge.waypoint_drive_node import WaypointDriveNode


def make_relative_pose_state(source, aruco_age, encoder_age=0.1):
    node = RelativePoseNode.__new__(RelativePoseNode)
    node.last_source = source
    node.pose_initialized_by_aruco = True
    node.last_accepted_aruco_time = aruco_age
    node.last_encoder_time = encoder_age
    node.aruco_fresh_timeout_sec = 0.5
    node.encoder_drive_grace_sec = 1.0
    node.encoder_fresh_timeout_sec = 0.5
    node.last_accepted_marker_count = 3
    node.good_min_marker_count = 3
    node.last_accepted_reproj_error = 1.0
    node.good_max_reproj_error_px = 3.0
    node.age_seconds = lambda stamp, now=None: stamp
    return node


def test_stop_keeps_recent_accepted_aruco_authority():
    node = make_relative_pose_state('stop', aruco_age=0.2)

    quality, usable, drive_allowed, _, _ = node.classify_pose_state(now=0.0)

    assert (quality, usable, drive_allowed) == ('GOOD', True, True)


def test_stop_becomes_hold_after_aruco_freshness_expires():
    node = make_relative_pose_state('stop', aruco_age=0.6)

    quality, usable, drive_allowed, _, _ = node.classify_pose_state(now=0.0)

    assert (quality, usable, drive_allowed) == ('HOLD', True, False)


def test_rejected_camera_frame_does_not_immediately_lose_recent_pose():
    node = make_relative_pose_state(
        'multi_marker_solvepnp', aruco_age=0.2
    )

    quality, usable, drive_allowed, _, _ = node.classify_pose_state(now=0.0)

    assert (quality, usable, drive_allowed) == ('GOOD', True, True)


def test_encoder_motion_remains_encoder_quality():
    node = make_relative_pose_state('encoder_turn', aruco_age=0.7)

    quality, usable, drive_allowed, _, _ = node.classify_pose_state(now=0.0)

    assert (quality, usable, drive_allowed) == ('ENCODER', True, True)


def test_navigation_pivot_grace_is_bounded():
    node = WaypointDriveNode.__new__(WaypointDriveNode)
    node.mode = 'NAV_PIVOT'
    node.last_navigation_cmd = 'q'
    node.last_aruco_control_time = 10.0
    node.nav_pivot_pose_grace_sec = 0.4
    node.elapsed = lambda start, end: end - start

    assert node.can_continue_navigation_pivot(10.39)
    assert not node.can_continue_navigation_pivot(10.41)


def make_follow_map_state():
    node = TurtlebotToF1FollowNode.__new__(TurtlebotToF1FollowNode)
    node.map_fallback_enabled = True
    node.static_map_planner = object()
    node.robot_pose_source = 'multi_marker_solvepnp'
    node.robot_pose_quality = 'OK'
    node.robot_pose_usable = True
    node.is_robot_pose_fresh = lambda now: True
    node.is_turtlebot_pose_fresh = lambda now: True
    return node


def test_map_fallback_available_with_usable_follower_and_fresh_leader():
    node = make_follow_map_state()

    assert node.is_map_fallback_available(now=0.0)


def test_map_fallback_stops_for_lost_or_unusable_follower_pose():
    node = make_follow_map_state()
    node.robot_pose_source = 'LOST'
    assert not node.is_map_fallback_available(now=0.0)

    node.robot_pose_source = 'stop'
    node.robot_pose_usable = False
    assert not node.is_map_fallback_available(now=0.0)


def test_map_fallback_stops_for_stale_leader_pose():
    node = make_follow_map_state()
    node.is_turtlebot_pose_fresh = lambda now: False

    assert not node.is_map_fallback_available(now=0.0)


def test_map_fallback_stops_without_static_map():
    node = make_follow_map_state()
    node.static_map_planner = None

    assert not node.is_map_fallback_available(now=0.0)


class FakePlanner:
    def __init__(self, result):
        self.result = result

    def plan_route(self, start_x, start_y, goal_x, goal_y):
        return self.result


def make_route_selection_state(result):
    node = TurtlebotToF1FollowNode.__new__(TurtlebotToF1FollowNode)
    node.static_map_planner = FakePlanner(result)
    node.robot_x = 0.0
    node.robot_y = 0.0
    node.map_target_x = 1.0
    node.map_target_y = 0.0
    node.map_line_free = False
    node.map_path_available = False
    node.map_blocked_reason = 'not_evaluated'
    node.map_path = []
    node.map_path_index = 0
    node.map_waypoint_reached_m = 0.10
    node.update_map_target_metrics = lambda: None
    return node


def test_follow_route_selection_uses_direct_only_for_free_line():
    result = {
        'status': 'direct',
        'reason': 'line_free',
        'line_free': True,
        'goal_x': 1.0,
        'goal_y': 0.0,
    }
    node = make_route_selection_state(result)

    mode, waypoint = node.prepare_map_fallback_route()

    assert mode == 'MAP_DIRECT'
    assert waypoint == (1.0, 0.0)
    assert node.map_line_free


def test_follow_route_selection_uses_astar_for_blocked_line():
    result = {
        'status': 'astar',
        'reason': 'line_blocked',
        'line_free': False,
        'path': [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)],
    }
    node = make_route_selection_state(result)

    mode, waypoint = node.prepare_map_fallback_route()

    assert mode == 'MAP_ASTAR'
    assert waypoint == (0.0, 1.0)
    assert node.map_path_available


def test_follow_route_selection_stops_when_astar_fails():
    result = {
        'status': 'failed',
        'reason': 'astar_failed',
        'line_free': False,
    }
    node = make_route_selection_state(result)

    mode, waypoint = node.prepare_map_fallback_route()

    assert mode is None
    assert waypoint is None
    assert node.map_blocked_reason == 'astar_failed'
