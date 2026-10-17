import unittest
from pathlib import Path

from multi_robot_sim.semantic_events import handle_event
from multi_robot_sim.semantic_validator import load_scene_graph


SCENE_PATH = (
    Path(__file__).resolve().parents[1]
    / "config" / "semantic" / "scene_graph.json"
)


class TestSemanticEvents(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.scene = load_scene_graph(SCENE_PATH)

    def setUp(self):
        self.request = "Inspect inspection_east and inspection_north"
        self.state = {
            "unavailable_robots": [],
            "unavailable_targets": [],
            "event_log": [],
        }

    def test_robot_failure_reassigns_tasks(self):
        plan, state, errors = handle_event(
            self.request,
            self.scene,
            {"type": "robot_unavailable", "value": "r0"},
            self.state,
        )
        self.assertEqual(errors, [])
        self.assertTrue(state["last_valid"])
        self.assertNotIn(
            "r0", [task["robot"] for task in plan["tasks"]]
        )
        self.assertEqual(state["unavailable_robots"], ["r0"])

    def test_unavailable_target_is_not_planned(self):
        plan, state, errors = handle_event(
            self.request,
            self.scene,
            {"type": "target_unavailable", "value": "inspection_east"},
            self.state,
        )
        self.assertEqual(errors, [])
        self.assertTrue(state["last_valid"])
        self.assertNotIn(
            "inspection_east",
            [task["target"] for task in plan["tasks"]],
        )
        self.assertEqual(
            state["unavailable_targets"], ["inspection_east"]
        )
        self.assertEqual(
            state["blocked_targets"], ["inspection_east"]
        )

    def test_unknown_event_rejected(self):
        plan, state, errors = handle_event(
            self.request,
            self.scene,
            {"type": "unexpected_event", "value": "x"},
            self.state,
        )
        self.assertIsNone(plan)
        self.assertTrue(errors)


if __name__ == "__main__":
    unittest.main()
