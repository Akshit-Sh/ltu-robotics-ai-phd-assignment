"""Ground natural-language mission requests in the configured scene graph."""

import re


def plan_mission(request, scene_graph, unavailable_robots=None):
    """Create a structured plan using configured locations and robots."""
    unavailable = set(unavailable_robots or [])
    request_lower = request.lower()

    robots = [
        robot["id"]
        for robot in scene_graph.get("robots", [])
        if robot["id"] not in unavailable
    ]
    locations = scene_graph.get("locations", [])

    if not robots:
        return {"tasks": []}, ["No available robots."]

    requested_locations = []
    for location in locations:
        location_id = location["id"]
        words = location_id.replace("_", " ")
        if location_id in request_lower or words in request_lower:
            requested_locations.append(location)

    if not requested_locations:
        return {"tasks": []}, [
            "No known location was grounded in the request."
        ]

    action = "inspect"
    if re.search(r"\b(observe|watch|look at)\b", request_lower):
        action = "observe"
    elif re.search(r"\b(go to|navigate to|move to)\b", request_lower):
        action = "navigate"

    tasks = []
    for index, location in enumerate(requested_locations):
        tasks.append({
            "task_id": f"semantic_{index + 1:03d}",
            "action": action,
            "target": location["id"],
            "robot": robots[index % len(robots)],
        })

    return {"tasks": tasks}, []


def main():
    """Run a demonstration of semantic planning and plan validation."""
    import json
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    from multi_robot_sim.semantic_validator import (
        load_scene_graph,
        validate_plan,
    )

    scene_path = (
        Path(get_package_share_directory("multi_robot_sim"))
        / "config" / "semantic" / "scene_graph.json"
    )
    scene = load_scene_graph(scene_path)
    request = "Inspect inspection_east and inspection_north"
    plan, planning_errors = plan_mission(request, scene)
    valid, validation_errors = validate_plan(plan, scene)

    print(json.dumps({
        "request": request,
        "plan": plan,
        "valid": valid,
        "planning_errors": planning_errors,
        "validation_errors": validation_errors,
    }, indent=2))


if __name__ == "__main__":
    main()
