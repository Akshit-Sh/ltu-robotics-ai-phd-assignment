"""Validation gate for structured semantic mission plans."""

import json
from pathlib import Path


SUPPORTED_ACTIONS = {"inspect", "observe", "navigate"}


def load_scene_graph(path):
    """Load the configured scene graph from JSON."""
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def validate_plan(plan, scene_graph, unavailable_robots=None):
    """Return (valid, errors) for a proposed mission plan."""
    unavailable = set(unavailable_robots or [])
    errors = []

    if not isinstance(plan, dict):
        return False, ["Plan must be a JSON object."]

    tasks = plan.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        return False, ["Plan must contain a non-empty tasks list."]

    known_robots = {robot["id"] for robot in scene_graph.get("robots", [])}
    known_locations = {
        location["id"] for location in scene_graph.get("locations", [])
    }
    task_ids = set()

    for index, task in enumerate(tasks):
        prefix = f"tasks[{index}]"

        if not isinstance(task, dict):
            errors.append(f"{prefix}: task must be an object.")
            continue

        task_id = task.get("task_id")
        if not isinstance(task_id, str) or not task_id.strip():
            errors.append(f"{prefix}: missing task_id.")
        elif task_id in task_ids:
            errors.append(f"{prefix}: duplicate task_id '{task_id}'.")
        else:
            task_ids.add(task_id)

        action = task.get("action")
        if action not in SUPPORTED_ACTIONS:
            errors.append(f"{prefix}: unsupported action '{action}'.")

        target = task.get("target")
        if target not in known_locations:
            errors.append(f"{prefix}: unknown target '{target}'.")

        robot = task.get("robot")
        if robot not in known_robots:
            errors.append(f"{prefix}: unknown robot '{robot}'.")
        elif robot in unavailable:
            errors.append(f"{prefix}: robot '{robot}' is unavailable.")

    return not errors, errors


def main():
    """Run a small command-line validation demonstration."""
    package_root = Path(__file__).resolve().parents[1]
    scene_path = (
        package_root / "config" / "semantic" / "scene_graph.json"
    )
    scene = load_scene_graph(scene_path)

    example_plan = {
        "tasks": [
            {
                "task_id": "task_001",
                "action": "inspect",
                "target": "inspection_east",
                "robot": "r0",
            },
            {
                "task_id": "task_002",
                "action": "inspect",
                "target": "inspection_north",
                "robot": "r1",
            },
        ]
    }

    valid, errors = validate_plan(example_plan, scene)
    print(json.dumps({
        "plan": example_plan,
        "valid": valid,
        "errors": errors,
    }, indent=2))


if __name__ == "__main__":
    main()
