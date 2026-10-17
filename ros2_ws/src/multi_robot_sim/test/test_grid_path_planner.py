"""Unit tests for SDF-derived A* planning."""

from pathlib import Path

import pytest

from multi_robot_sim.grid_path_planner import (
    AStarGridPlanner,
    BoxObstacle,
)


def test_sdf_world_loads_six_box_obstacles():
    world = (
        Path(__file__).resolve().parents[1]
        / "worlds" / "inspection_world.world"
    )
    planner = AStarGridPlanner.from_sdf(world)
    assert len(planner.obstacles) == 6


def test_path_avoids_inflated_rectangular_obstacle():
    planner = AStarGridPlanner(
        [BoxObstacle(0.0, 0.0, 0.5, 1.5)],
        resolution=0.1,
        bounds=(-2.0, 2.0, -2.0, 2.0),
        inflation=0.15,
    )
    path = planner.plan((-1.0, 0.0), (1.0, 0.0))
    assert path is not None
    assert len(path) >= 3
    assert any(abs(y) > 0.9 for _, y in path)


def test_unreachable_goal_returns_none():
    planner = AStarGridPlanner(
        [BoxObstacle(0.0, 0.0, 0.5, 3.0)],
        resolution=0.1,
        bounds=(-1.0, 1.0, -1.0, 1.0),
        inflation=0.0,
    )
    assert planner.plan((-0.8, 0.0), (0.8, 0.0)) is None


def test_goal_inside_inflated_obstacle_is_rejected():
    planner = AStarGridPlanner(
        [BoxObstacle(0.0, 0.0, 0.5, 0.5)],
        resolution=0.1,
        bounds=(-2.0, 2.0, -2.0, 2.0),
        inflation=0.2,
    )
    with pytest.raises(ValueError, match="Goal"):
        planner.plan((-1.0, 0.0), (0.0, 0.0))


def test_all_mission_goals_are_reachable_from_spawn_points():
    world = (
        Path(__file__).resolve().parents[1]
        / "worlds" / "inspection_world.world"
    )
    planner = AStarGridPlanner.from_sdf(world)
    starts = ((0.0, 0.0), (2.0, 0.0), (0.0, 2.0))
    goals = (
        (1.0, 1.0), (2.0, 1.0), (3.0, 1.0), (3.0, 2.0),
        (2.0, 3.0), (1.0, 3.0), (-1.0, 3.0), (-2.0, 2.0),
        (-2.0, 1.0), (-1.0, -1.0),
    )
    for start in starts:
        for goal in goals:
            assert planner.plan(start, goal) is not None


def test_sdf_path_detours_around_existing_block_b():
    """Verify A* detours around block_b in the actual Gazebo world."""
    world = (
        Path(__file__).resolve().parents[1]
        / "worlds" / "inspection_world.world"
    )
    planner = AStarGridPlanner.from_sdf(world)

    start = (1.4, -3.4)
    goal = (1.4, -1.6)
    path = planner.plan(start, goal)

    assert path is not None
    assert planner.is_blocked(planner.world_to_grid(1.4, -2.5))
    assert max(abs(x - start[0]) for x, _ in path) > 0.15
