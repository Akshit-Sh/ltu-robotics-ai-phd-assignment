"""Tests for LLM semantic-plan grounding and validation."""

import unittest
from pathlib import Path

from multi_robot_sim.llm_semantic_planner import (
    _normalise_and_ground,
)
from multi_robot_sim.semantic_validator import load_scene_graph


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = PACKAGE_ROOT / "config" / "semantic" / "scene_graph.json"


class TestLLMSemanticPlanner(unittest.TestCase):
    """Test model-output grounding without requiring Ollama."""

    @classmethod
    def setUpClass(cls):
        cls.scene = load_scene_graph(SCENE_PATH)

    def test_valid_model_output_is_grounded(self):
        raw = {
            "tasks": [{
                "task_id": "task_001",
                "action": "Inspect",
                "target": "east inspection area",
                "robot_id": "r0",
            }]
        }

        plan, errors = _normalise_and_ground(raw, self.scene)

        self.assertEqual(errors, [])
        self.assertEqual(plan["tasks"][0]["target"], "inspection_east")
        self.assertEqual(plan["tasks"][0]["action"], "inspect")
        self.assertEqual(plan["tasks"][0]["robot"], "r0")

    def test_unknown_target_is_rejected(self):
        raw = {
            "tasks": [{
                "task_id": "task_001",
                "action": "inspect",
                "target": "made_up_room",
                "robot": "r0",
            }]
        }

        plan, errors = _normalise_and_ground(raw, self.scene)

        self.assertEqual(plan["tasks"], [])
        self.assertTrue(errors)
        self.assertTrue(any("does not exactly match" in e for e in errors))

    def test_empty_tasks_are_rejected(self):
        plan, errors = _normalise_and_ground({"tasks": []}, self.scene)

        self.assertEqual(plan["tasks"], [])
        self.assertTrue(errors)

    def test_malformed_task_is_rejected(self):
        plan, errors = _normalise_and_ground(
            {"tasks": ["not a task object"]}, self.scene
        )

        self.assertEqual(plan["tasks"], [])
        self.assertTrue(any("must be an object" in e for e in errors))

    def test_unsupported_action_is_rejected(self):
        raw = {
            "tasks": [{
                "task_id": "task_001",
                "action": "dance",
                "target": "inspection_east",
                "robot": "r0",
            }]
        }

        plan, errors = _normalise_and_ground(raw, self.scene)

        self.assertEqual(plan["tasks"], [])
        self.assertTrue(any("unsupported action" in e for e in errors))

    def test_unavailable_robots_are_not_assigned(self):
        raw = {
            "tasks": [{
                "task_id": "task_001",
                "action": "inspect",
                "target": "inspection_east",
                "robot": "r0",
            }]
        }

        plan, errors = _normalise_and_ground(
            raw, self.scene, unavailable_robots=["r0"]
        )

        self.assertEqual(errors, [])
        self.assertNotEqual(plan["tasks"][0]["robot"], "r0")

    def test_no_available_robots_rejects_plan(self):
        plan, errors = _normalise_and_ground(
            {"tasks": [{
                "task_id": "task_001",
                "action": "inspect",
                "target": "inspection_east",
                "robot": "r0",
            }]},
            self.scene,
            unavailable_robots=["r0", "r1", "r2"],
        )

        self.assertEqual(plan["tasks"], [])
        self.assertTrue(any("No available robots" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
