"""Scripted event handling and semantic mission replanning."""

from copy import deepcopy

from multi_robot_sim.semantic_planner import plan_mission
from multi_robot_sim.semantic_validator import validate_plan


def handle_event(request, scene_graph, event, state):
    """Apply an event, replan, validate, and return updated state."""
    updated_scene = deepcopy(scene_graph)
    unavailable_robots = set(state.get("unavailable_robots", []))
    unavailable_targets = set(state.get("unavailable_targets", []))
    event_log = list(state.get("event_log", []))

    event_type = event.get("type")
    event_value = event.get("value")

    if event_type == "robot_unavailable":
        unavailable_robots.add(event_value)
        event_log.append({
            "type": event_type,
            "value": event_value,
            "result": "robot marked unavailable",
        })
    elif event_type == "target_unavailable":
        unavailable_targets.add(event_value)
        event_log.append({
            "type": event_type,
            "value": event_value,
            "result": "target marked unavailable",
        })
    else:
        return None, state, [f"Unsupported event type: {event_type}"]

    # Record blocked targets so omitted work is not mistaken for completion.
    blocked_targets = sorted(unavailable_targets)

    # Remove unavailable targets from the grounding graph.
    updated_scene["locations"] = [
        location for location in updated_scene.get("locations", [])
        if location["id"] not in unavailable_targets
    ]

    plan, planning_errors = plan_mission(
        request, updated_scene, sorted(unavailable_robots)
    )
    valid, validation_errors = validate_plan(
        plan, updated_scene, sorted(unavailable_robots)
    )

    new_state = {
        "unavailable_robots": sorted(unavailable_robots),
        "unavailable_targets": sorted(unavailable_targets),
        "blocked_targets": blocked_targets,
        "event_log": event_log,
        "last_plan": plan,
        "last_valid": valid,
    }
    return plan, new_state, planning_errors + validation_errors


def main():
    """Demonstrate replanning after two scripted events."""
    import json
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    from multi_robot_sim.semantic_validator import load_scene_graph

    scene_path = (
        Path(get_package_share_directory("multi_robot_sim"))
        / "config" / "semantic" / "scene_graph.json"
    )
    scene = load_scene_graph(scene_path)
    request = "Inspect inspection_east and inspection_north"
    state = {
        "unavailable_robots": [],
        "unavailable_targets": [],
        "event_log": [],
    }
    results = []

    for event in [
        {"type": "robot_unavailable", "value": "r0"},
        {"type": "target_unavailable", "value": "inspection_east"},
    ]:
        plan, state, errors = handle_event(
            request, scene, event, state
        )
        results.append({
            "event": event,
            "plan": plan,
            "errors": errors,
            "state": state,
        })

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
