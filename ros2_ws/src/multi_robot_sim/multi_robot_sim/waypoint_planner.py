
"""Deterministic waypoint allocation for the multi-robot assignment."""

from collections import defaultdict

ROBOT_IDS = ("r0", "r1", "r2")

# Coordinates are in metres in the Gazebo world frame.
# These are mission targets, not yet validated navigation goals.
WAYPOINTS = (
    ("wp01", 1.0, 1.0),
    ("wp02", 2.0, 1.0),
    ("wp03", 3.0, 1.0),
    ("wp04", 3.0, 2.0),
    ("wp05", 2.0, 3.0),
    ("wp06", 1.0, 3.0),
    ("wp07", -1.0, 3.0),
    ("wp08", -2.0, 2.0),
    ("wp09", -2.0, 1.0),
    ("wp10", -1.0, -1.0),
)


def allocate_waypoints(
    waypoints=WAYPOINTS,
    robot_ids=ROBOT_IDS,
    completed=(),
    failed_robots=(),
):
    """
    Assign every uncompleted waypoint once among available robots.

    Returns a dict mapping robot IDs to lists of waypoint tuples.
    Completed waypoint IDs are never reassigned.
    """
    robot_ids = tuple(robot_ids)
    failed_robots = set(failed_robots)
    completed = set(completed)

    active_robots = [
        robot for robot in robot_ids if robot not in failed_robots
    ]

    if not active_robots:
        raise ValueError("At least one robot must remain available.")

    waypoint_ids = [wp[0] for wp in waypoints]
    if len(waypoint_ids) != len(set(waypoint_ids)):
        raise ValueError("Waypoint IDs must be unique.")

    assignments = defaultdict(list)
    remaining = [
        wp for wp in waypoints if wp[0] not in completed
    ]

    for index, waypoint in enumerate(remaining):
        robot = active_robots[index % len(active_robots)]
        assignments[robot].append(waypoint)

    return {
        robot: assignments[robot]
        for robot in robot_ids
        if robot in active_robots
    }


def validate_assignments(assignments, waypoints=WAYPOINTS, completed=()):
    """Raise ValueError if remaining targets are missing or duplicated."""
    expected = {
        wp[0] for wp in waypoints if wp[0] not in set(completed)
    }
    assigned = [
        wp[0]
        for robot_waypoints in assignments.values()
        for wp in robot_waypoints
    ]

    if len(assigned) != len(set(assigned)):
        raise ValueError("A waypoint has been assigned more than once.")

    if set(assigned) != expected:
        raise ValueError("Assignments do not match the remaining waypoints.")

    return True


def main(args=None):
    """Publish the current waypoint allocation on a ROS 2 topic."""
    import json

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class WaypointPlannerNode(Node):
        """Publish centralized waypoint assignments periodically."""

        def __init__(self):
            super().__init__("central_waypoint_planner")
            self.publisher = self.create_publisher(
                String, "/mission/assignments", 10
            )
            self.timer = self.create_timer(2.0, self.publish_plan)

        def publish_plan(self):
            """Publish the current allocation as a JSON message."""
            plan = allocate_waypoints()

            payload = {
                robot: [
                    {"id": wp_id, "x": x, "y": y}
                    for wp_id, x, y in waypoints
                ]
                for robot, waypoints in plan.items()
            }

            message = String()
            message.data = json.dumps(payload, sort_keys=True)
            self.publisher.publish(message)

            self.get_logger().info(
                f"Published {sum(len(v) for v in plan.values())} "
                "unique waypoint assignments."
            )

    rclpy.init(args=args)
    node = WaypointPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
