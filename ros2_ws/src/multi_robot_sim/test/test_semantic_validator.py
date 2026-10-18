import unittest
from pathlib import Path

from multi_robot_sim.semantic_validator import (
    extract_requested_targets,
    load_scene_graph,
    validate_plan,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = PACKAGE_ROOT / "config" / "semantic" / "scene_graph.json"


class TestSemanticValidator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.scene = load_scene_graph(SCENE_PATH)

    def make_plan(self, **overrides):
        task = {
            "task_id": "task_001",
            "action": "inspect",
            "target": "inspection_east",
            "robot": "r0",
        }
        task.update(overrides)
        return {"tasks": [task]}

    def test_valid_plan(self):
        valid, errors = validate_plan(self.make_plan(), self.scene)
        self.assertTrue(valid)
        self.assertEqual(errors, [])

    def test_unknown_target_rejected(self):
        valid, errors = validate_plan(
            self.make_plan(target="unknown_place"), self.scene
        )
        self.assertFalse(valid)
        self.assertTrue(any("unknown target" in error for error in errors))

    def test_unsupported_action_rejected(self):
        valid, errors = validate_plan(
            self.make_plan(action="dance"), self.scene
        )
        self.assertFalse(valid)
        self.assertTrue(any("unsupported action" in error for error in errors))

    def test_unknown_robot_rejected(self):
        valid, errors = validate_plan(
            self.make_plan(robot="r9"), self.scene
        )
        self.assertFalse(valid)
        self.assertTrue(any("unknown robot" in error for error in errors))

    def test_unavailable_robot_rejected(self):
        valid, errors = validate_plan(
            self.make_plan(), self.scene, unavailable_robots=["r0"]
        )
        self.assertFalse(valid)
        self.assertTrue(any("unavailable" in error for error in errors))

    def test_duplicate_task_ids_rejected(self):
        plan = self.make_plan()
        plan["tasks"].append({
            "task_id": "task_001",
            "action": "observe",
            "target": "inspection_north",
            "robot": "r1",
        })
        valid, errors = validate_plan(plan, self.scene)
        self.assertFalse(valid)
        self.assertTrue(any("duplicate task_id" in error for error in errors))

    def test_explicit_requested_targets_extracted(self):
        targets = extract_requested_targets(
            "Inspect inspection_east and the north inspection area.",
            self.scene,
        )
        self.assertEqual(
            set(targets), {"inspection_east", "inspection_north"}
        )

    def test_all_inspection_areas_are_required(self):
        targets = extract_requested_targets(
            "Inspect all inspection areas.", self.scene
        )
        expected = {
            item["id"] for item in self.scene["locations"]
            if item.get("type") == "inspection_area"
        }
        self.assertTrue(expected)
        self.assertEqual(set(targets), expected)

    def test_missing_required_target_is_rejected(self):
        valid, errors = validate_plan(
            self.make_plan(),
            self.scene,
            required_targets=["inspection_east", "inspection_north"],
        )
        self.assertFalse(valid)
        self.assertTrue(any(
            "omits requested target 'inspection_north'" in error
            for error in errors
        ))

    def test_two_robots_cannot_own_same_target(self):
        plan = self.make_plan()
        plan["tasks"].append({
            "task_id": "task_002",
            "action": "observe",
            "target": "inspection_east",
            "robot": "r1",
        })
        valid, errors = validate_plan(plan, self.scene)
        self.assertFalse(valid)
        self.assertTrue(any(
            "assigned to multiple robots" in error for error in errors
        ))


if __name__ == "__main__":
    unittest.main()
