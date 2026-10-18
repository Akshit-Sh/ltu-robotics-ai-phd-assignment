"""Regression tests for semantic planning, grounding and events."""

from pathlib import Path

import pytest

from multi_robot_sim.llm_semantic_planner import (
    _normalise_and_ground,
    plan_mission_with_llm,
)
from multi_robot_sim.semantic_events import handle_event
from multi_robot_sim.semantic_planner import plan_mission
from multi_robot_sim.semantic_validator import load_scene_graph, validate_plan


SCENE_PATH = (
    Path(__file__).resolve().parents[1]
    / "config" / "semantic" / "scene_graph.json"
)


@pytest.fixture
def scene():
    return load_scene_graph(SCENE_PATH)


def test_rule_planner_grounds_known_locations(scene):
    plan, errors = plan_mission(
        "Inspect inspection_east and inspection_north", scene
    )
    assert errors == []
    assert [task["target"] for task in plan["tasks"]] == [
        "inspection_east", "inspection_north"
    ]
    assert all(
        task["robot"] in {"r0", "r1", "r2"}
        for task in plan["tasks"]
    )


def test_rule_planner_rejects_unknown_location(scene):
    plan, errors = plan_mission("Inspect the green rooftop", scene)
    assert plan == {"tasks": []}
    assert errors


def test_validator_rejects_duplicate_task_ids(scene):
    task = {
        "task_id": "same",
        "action": "inspect",
        "target": "inspection_east",
        "robot": "r0",
    }
    valid, errors = validate_plan({"tasks": [task, dict(task)]}, scene)
    assert not valid
    assert any("duplicate task_id" in error for error in errors)


def test_validator_rejects_unknown_action_target_and_robot(scene):
    plan = {"tasks": [{
        "task_id": "bad",
        "action": "fly",
        "target": "imaginary_room",
        "robot": "r99",
    }]}
    valid, errors = validate_plan(plan, scene)
    assert not valid
    assert len(errors) == 3


def test_validator_handles_unhashable_field_types(scene):
    plan = {"tasks": [{
        "task_id": "malformed",
        "action": [],
        "target": [],
        "robot": [],
    }]}
    valid, errors = validate_plan(plan, scene)
    assert not valid
    assert len(errors) == 3


def test_validator_rejects_unavailable_robot(scene):
    plan = {"tasks": [{
        "task_id": "task_1",
        "action": "inspect",
        "target": "inspection_east",
        "robot": "r0",
    }]}
    valid, errors = validate_plan(
        plan, scene, unavailable_robots={"r0"}
    )
    assert not valid
    assert any("unavailable" in error for error in errors)


def test_llm_grounding_maps_known_alias(scene):
    raw_plan = {"tasks": [{
        "task_id": "task_1",
        "action": "inspect",
        "target": "east inspection area",
        "robot": "r0",
    }]}
    plan, errors = _normalise_and_ground(raw_plan, scene)
    assert errors == []
    assert plan["tasks"][0]["target"] == "inspection_east"


def test_llm_grounding_rejects_invented_target(scene):
    raw_plan = {"tasks": [{
        "task_id": "task_1",
        "action": "inspect",
        "target": "secret laboratory",
        "robot": "r0",
    }]}
    plan, errors = _normalise_and_ground(raw_plan, scene)
    assert plan["tasks"] == []
    assert errors


def test_ambiguous_location_requires_clarification(scene, monkeypatch):
    def unexpected_model_call(*args, **kwargs):
        raise AssertionError("Ambiguous request must be intercepted before Ollama")

    monkeypatch.setattr(
        "multi_robot_sim.llm_semantic_planner.requests.post",
        unexpected_model_call,
    )
    plan, result = plan_mission_with_llm(
        "Inspect the appropriate area using robot r0.", scene
    )

    assert plan == {"tasks": []}
    assert result["source"] == "ambiguity_guard"
    assert result["valid"] is False
    assert result["requires_clarification"] is True
    assert set(result["ambiguity_candidates"]) == {
        "inspection_east",
        "inspection_north",
        "inspection_west",
        "inspection_south",
    }
    assert any("Ambiguous location" in error for error in result["errors"])


def test_unique_location_does_not_trigger_ambiguity_guard(scene):
    from multi_robot_sim.llm_semantic_planner import _detect_ambiguous_request

    assert _detect_ambiguous_request(
        "Inspect the east inspection area using robot r0.", scene
    ) == []


def test_unknown_target_is_not_mislabeled_as_ambiguous(scene):
    from multi_robot_sim.llm_semantic_planner import _detect_ambiguous_request

    assert _detect_ambiguous_request(
        "Inspect the green rooftop.", scene
    ) == []


def test_robot_unavailable_event_replans(scene):
    request = "Inspect inspection_east and inspection_north"
    state = {
        "unavailable_robots": [],
        "unavailable_targets": [],
        "event_log": [],
    }
    plan, state, errors = handle_event(
        request, scene,
        {"type": "robot_unavailable", "value": "r0"}, state
    )
    assert errors == []
    assert state["last_valid"]
    assert plan["tasks"]
    assert all(task["robot"] != "r0" for task in plan["tasks"])


def test_target_unavailable_event_replans(scene):
    request = "Inspect inspection_east and inspection_north"
    state = {
        "unavailable_robots": [],
        "unavailable_targets": [],
        "event_log": [],
    }
    _, state, errors = handle_event(
        request, scene,
        {"type": "robot_unavailable", "value": "r0"}, state
    )
    assert errors == []

    plan, state, errors = handle_event(
        request, scene,
        {"type": "target_unavailable", "value": "inspection_east"},
        state,
    )
    assert errors == []
    assert state["last_valid"]
    assert "inspection_east" in state["blocked_targets"]
    assert all(
        task["target"] != "inspection_east" for task in plan["tasks"]
    )
    assert any(
        task["target"] == "inspection_north" for task in plan["tasks"]
    )


def test_unknown_event_is_rejected_without_changing_state(scene):
    state = {
        "unavailable_robots": [],
        "unavailable_targets": [],
        "event_log": [],
    }
    plan, new_state, errors = handle_event(
        "Inspect inspection_east", scene,
        {"type": "teleport", "value": "r0"}, state
    )
    assert plan is None
    assert new_state is state
    assert errors


def test_scene_hierarchy_covers_configured_entities(scene):
    from multi_robot_sim.semantic_validator import validate_scene_hierarchy

    valid, errors = validate_scene_hierarchy(scene)
    assert valid, errors


def test_validator_rejects_dangling_scene_hierarchy_reference(scene):
    from copy import deepcopy

    malformed_scene = deepcopy(scene)
    location_group = next(
        group for group in malformed_scene["hierarchy"]["children"]
        if group["entity_type"] == "location"
    )
    location_group["children"].append("invented_location")

    plan = {"tasks": [{
        "task_id": "task_1",
        "action": "inspect",
        "target": "inspection_east",
        "robot": "r0",
    }]}
    valid, errors = validate_plan(plan, malformed_scene)
    assert not valid
    assert any("unknown location" in error for error in errors)
