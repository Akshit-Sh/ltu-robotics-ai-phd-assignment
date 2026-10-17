"""Validation gate for structured semantic mission plans."""

import json
from pathlib import Path


SUPPORTED_ACTIONS = {"inspect", "observe", "navigate"}


def load_scene_graph(path):
    """Load the configured scene graph from JSON."""
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def validate_scene_hierarchy(scene_graph):
    """Ensure hierarchy leaves refer to configured robots, locations and objects."""
    errors = []
    if not isinstance(scene_graph, dict):
        return False, ["Scene graph must be a JSON object."]

    root = scene_graph.get("hierarchy")
    if not isinstance(root, dict):
        return False, ["Scene graph must contain a hierarchy object."]
    if root.get("id") != scene_graph.get("world"):
        errors.append("Hierarchy root must match the configured world.")
    if root.get("node_kind") != "world":
        errors.append("Hierarchy root node_kind must be 'world'.")

    collections = {
        "robot": "robots",
        "location": "locations",
        "object": "objects",
    }
    entity_ids = {}
    for entity_type, collection_name in collections.items():
        records = scene_graph.get(collection_name, [])
        ids = set()
        if not isinstance(records, list):
            errors.append(f"'{collection_name}' must be a list.")
            entity_ids[entity_type] = ids
            continue
        for index, record in enumerate(records):
            if not isinstance(record, dict):
                errors.append(
                    f"{collection_name}[{index}] must be an object."
                )
                continue
            entity_id = record.get("id")
            if not isinstance(entity_id, str) or not entity_id.strip():
                errors.append(
                    f"{collection_name}[{index}] has no valid id."
                )
            elif entity_id in ids:
                errors.append(
                    f"Duplicate entity id '{entity_id}' in {collection_name}."
                )
            else:
                ids.add(entity_id)
        entity_ids[entity_type] = ids

    groups = root.get("children")
    if not isinstance(groups, list):
        return False, errors + ["Hierarchy children must be a list."]

    seen_types = set()
    seen_entities = set()
    for index, group in enumerate(groups):
        if not isinstance(group, dict):
            errors.append(f"hierarchy.children[{index}] must be an object.")
            continue
        entity_type = group.get("entity_type")
        if group.get("node_kind") != "group":
            errors.append(f"Hierarchy child {index} must be a group.")
        if entity_type not in collections:
            errors.append(
                f"Hierarchy group {index} has an unsupported entity_type."
            )
            continue
        if entity_type in seen_types:
            errors.append(
                f"Duplicate hierarchy group for '{entity_type}'."
            )
        seen_types.add(entity_type)

        children = group.get("children")
        if not isinstance(children, list):
            errors.append(
                f"Hierarchy group '{entity_type}' children must be a list."
            )
            continue
        for child in children:
            if not isinstance(child, str) or child not in entity_ids[entity_type]:
                errors.append(
                    f"Hierarchy references unknown {entity_type} '{child}'."
                )
                continue
            reference = (entity_type, child)
            if reference in seen_entities:
                errors.append(
                    f"Duplicate hierarchy reference '{child}'."
                )
            seen_entities.add(reference)

    if seen_types != set(collections):
        missing = sorted(set(collections) - seen_types)
        errors.append(f"Missing hierarchy groups: {missing}.")

    expected_entities = {
        (entity_type, entity_id)
        for entity_type, ids in entity_ids.items()
        for entity_id in ids
    }
    if seen_entities != expected_entities:
        missing = sorted(expected_entities - seen_entities)
        errors.append(f"Entities missing from hierarchy: {missing}.")

    return not errors, errors


def validate_plan(plan, scene_graph, unavailable_robots=None):
    """Return (valid, errors) for a proposed mission plan."""
    if isinstance(scene_graph, dict) and "hierarchy" in scene_graph:
        hierarchy_valid, hierarchy_errors = validate_scene_hierarchy(
            scene_graph
        )
        if not hierarchy_valid:
            return False, [
                "Invalid scene hierarchy: " + error
                for error in hierarchy_errors
            ]

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
        if (not isinstance(action, str)
                or action not in SUPPORTED_ACTIONS):
            errors.append(f"{prefix}: unsupported action '{action}'.")

        target = task.get("target")
        if (not isinstance(target, str)
                or target not in known_locations):
            errors.append(f"{prefix}: unknown target '{target}'.")

        robot = task.get("robot")
        if (not isinstance(robot, str)
                or robot not in known_robots):
            errors.append(f"{prefix}: unknown robot '{robot}'.")
        elif robot in unavailable:
            errors.append(f"{prefix}: robot '{robot}' is unavailable.")

    return not errors, errors


def main():
    """Run a small command-line validation demonstration."""
    from ament_index_python.packages import get_package_share_directory

    scene_path = (
        Path(get_package_share_directory("multi_robot_sim"))
        / "config" / "semantic" / "scene_graph.json"
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
