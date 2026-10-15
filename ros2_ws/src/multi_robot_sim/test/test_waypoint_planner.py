"""Unit tests for centralized multi-robot waypoint allocation."""

import pytest

from multi_robot_sim.waypoint_planner import (
    ROBOT_IDS,
    WAYPOINTS,
    allocate_waypoints,
    validate_assignments,
)


def test_all_waypoints_are_assigned_exactly_once():
    assignments = allocate_waypoints()

    assert validate_assignments(assignments)
    assigned_ids = [
        wp[0]
        for robot_waypoints in assignments.values()
        for wp in robot_waypoints
    ]

    assert len(assigned_ids) == len(WAYPOINTS)
    assert len(assigned_ids) == len(set(assigned_ids))


def test_completed_waypoints_are_not_reassigned():
    completed = ("wp01", "wp04")
    assignments = allocate_waypoints(completed=completed)

    assert validate_assignments(assignments, completed=completed)

    assigned_ids = {
        wp[0]
        for robot_waypoints in assignments.values()
        for wp in robot_waypoints
    }
    assert not set(completed).intersection(assigned_ids)


def test_failed_robot_receives_no_assignments():
    assignments = allocate_waypoints(failed_robots=("r1",))

    assert "r1" not in assignments
    assert set(assignments) == {"r0", "r2"}
    assert validate_assignments(assignments)


def test_all_robots_failed_raises_value_error():
    with pytest.raises(ValueError, match="At least one robot"):
        allocate_waypoints(failed_robots=ROBOT_IDS)


def test_duplicate_waypoint_ids_are_rejected():
    duplicate_waypoints = (
        ("wp01", 1.0, 1.0),
        ("wp01", 2.0, 2.0),
    )

    with pytest.raises(ValueError, match="Waypoint IDs must be unique"):
        allocate_waypoints(waypoints=duplicate_waypoints)
