import numpy as np

from encoder_bridge.static_map_planner import StaticMapPlanner


def make_planner():
    planner = StaticMapPlanner.__new__(StaticMapPlanner)
    planner.resolution = 1.0
    planner.origin_x = 0.0
    planner.origin_y = 0.0
    planner.width = 10
    planner.height = 10
    planner.inflated_grid = np.zeros((10, 10), dtype=np.uint8)
    planner.allow_diagonal = True
    planner.goal_search_radius_m = 3.0
    planner.waypoint_spacing_m = 1.0
    return planner


def block_cell(planner, x, y):
    row, col = planner.grid_to_img(x, y)
    planner.inflated_grid[row, col] = 1


def test_direct_route_is_allowed_only_when_inflated_line_is_free():
    planner = make_planner()

    result = planner.plan_route(1.5, 1.5, 8.5, 1.5)

    assert result['status'] == 'direct'
    assert result['line_free']


def test_blocked_line_uses_astar_detour():
    planner = make_planner()
    for y in range(8):
        block_cell(planner, 4, y)

    result = planner.plan_route(1.5, 1.5, 8.5, 1.5)

    assert result['status'] == 'astar'
    assert not result['line_free']
    assert result['reason'] == 'line_blocked'
    assert len(result['path']) > 2


def test_astar_failure_stops_route():
    planner = make_planner()
    for y in range(10):
        block_cell(planner, 4, y)

    result = planner.plan_route(1.5, 1.5, 8.5, 1.5)

    assert result['status'] == 'failed'
    assert result['reason'] == 'astar_failed'


def test_goal_in_obstacle_uses_nearest_free_goal():
    planner = make_planner()
    block_cell(planner, 8, 1)

    result = planner.plan_route(1.5, 1.5, 8.5, 1.5)

    assert result['status'] == 'astar'
    assert result['goal_adjusted']
    assert result['reason'] == 'goal_adjusted'


def test_start_in_inflated_obstacle_is_rejected():
    planner = make_planner()
    block_cell(planner, 1, 1)

    result = planner.plan_route(1.5, 1.5, 8.5, 1.5)

    assert result['status'] == 'failed'
    assert result['reason'] == 'start_in_inflated_obstacle'
