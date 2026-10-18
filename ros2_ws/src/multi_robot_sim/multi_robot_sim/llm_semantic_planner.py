"""LLM-backed semantic planner with scene-graph grounding and validation."""

import json
from pathlib import Path

import requests

from multi_robot_sim.semantic_planner import plan_mission
from multi_robot_sim.semantic_validator import (
    extract_requested_targets,
    load_scene_graph,
    validate_plan,
)

OLLAMA_URL = "http://127.0.0.1:11434/api/generate"
MODEL_NAME = "qwen2.5:1.5b"


def _normalise_and_ground(raw_plan, scene_graph, unavailable_robots=None):
    """Normalise model output and ground it against known scene entities."""
    unavailable = set(unavailable_robots or [])
    robots = [
        item["id"] for item in scene_graph.get("robots", [])
        if item["id"] not in unavailable
    ]
    locations = {
        item["id"]: item for item in scene_graph.get("locations", [])
    }
    location_aliases = {}
    for location_id in locations:
        location_aliases[location_id.lower()] = location_id
        location_aliases[location_id.replace("_", " ").lower()] = location_id
    location_aliases.update({
        "east inspection area": "inspection_east",
        "north inspection area": "inspection_north",
        "west inspection area": "inspection_west",
        "south inspection area": "inspection_south",
        "starting area": "inspection_origin",
        "origin": "inspection_origin",
    })

    if not robots:
        return {"tasks": []}, ["No available robots."]

    tasks = raw_plan.get("tasks") if isinstance(raw_plan, dict) else None
    if not isinstance(tasks, list) or not tasks:
        return {"tasks": []}, ["Model output must contain a non-empty tasks list."]

    grounded = []
    errors = []
    for index, task in enumerate(tasks):
        if not isinstance(task, dict):
            errors.append(f"tasks[{index}] must be an object.")
            continue

        target_text = str(task.get("target", "")).strip().lower()
        target = location_aliases.get(target_text)
        if target is None:
            errors.append(
                f"tasks[{index}]: target '{task.get('target')}' "
                "does not exactly match a known location or alias."
            )
            continue

        action = str(task.get("action", "")).strip().lower()
        if action not in {"inspect", "observe", "navigate"}:
            errors.append(f"tasks[{index}]: unsupported action '{action}'.")
            continue
        requested_robot = task.get("robot", task.get("robot_id"))
        if requested_robot not in robots:
            errors.append(
                f"tasks[{index}]: robot '{requested_robot}' is "
                "unknown or unavailable."
            )
            continue

        grounded.append({
            "task_id": str(task.get("task_id") or f"llm_{index + 1:03d}"),
            "action": action,
            "target": target,
            "robot": requested_robot,
        })

    if errors:
        return {"tasks": grounded}, errors
    return {"tasks": grounded}, []


def plan_mission_with_llm(
    request, scene_graph, unavailable_robots=None, timeout=60
):
    """Generate, ground, and validate a plan using local Ollama."""
    prompt = f"""
You convert a robot mission into a JSON task plan.
Return JSON only, with this exact structure:
{{"tasks":[{{"task_id":"task_001","action":"inspect",
"target":"inspection_east","robot":"r0"}}]}}
Allowed actions: inspect, observe, navigate.
Use only these exact location IDs: {
    ", ".join(item["id"] for item in scene_graph.get("locations", []))
}
Use only these robot IDs: {
    ", ".join(item["id"] for item in scene_graph.get("robots", []))
}
Do not invent targets. If the request is ambiguous or no target is known,
return {{"tasks":[]}}.
Mission: {request}
""".strip()

    try:
        response = requests.post(
            OLLAMA_URL,
            json={
                "model": MODEL_NAME,
                "prompt": prompt,
                "stream": False,
                "format": "json",
                "options": {"temperature": 0},
            },
            timeout=timeout,
        )
        response.raise_for_status()
        content = response.json().get("response", "")
        raw_plan = json.loads(content)
    except (requests.RequestException, ValueError, TypeError) as exc:
        fallback_plan, fallback_errors = plan_mission(
            request, scene_graph, unavailable_robots
        )
        valid, validation_errors = validate_plan(
            fallback_plan, scene_graph, unavailable_robots,
            required_targets=extract_requested_targets(
                request, scene_graph
            ),
        )
        return fallback_plan, {
            "source": "deterministic_fallback",
            "model_error": str(exc),
            "errors": fallback_errors + validation_errors,
            "valid": valid and not fallback_errors,
        }

    plan, grounding_errors = _normalise_and_ground(
        raw_plan, scene_graph, unavailable_robots
    )
    valid, validation_errors = validate_plan(
        plan, scene_graph, unavailable_robots,
        required_targets=extract_requested_targets(
            request, scene_graph
        ),
    )
    errors = grounding_errors + validation_errors

    return plan, {
        "source": "ollama",
        "model": MODEL_NAME,
        "valid": not errors and valid,
        "errors": errors,
        "raw_output": raw_plan,
    }


def main():
    """Run a local LLM semantic-planning demonstration."""
    from ament_index_python.packages import get_package_share_directory

    scene_path = (
        Path(get_package_share_directory("multi_robot_sim"))
        / "config" / "semantic" / "scene_graph.json"
    )
    scene = load_scene_graph(scene_path)
    request = "Inspect the east inspection area using robot r0."
    plan, result = plan_mission_with_llm(request, scene)
    print(json.dumps({
        "request": request,
        "plan": plan,
        "result": result,
    }, indent=2))


if __name__ == "__main__":
    main()
