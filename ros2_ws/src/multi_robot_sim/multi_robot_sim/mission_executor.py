"""Centralized three-robot waypoint mission executor."""

import json
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from multi_robot_sim.waypoint_planner import (
    ROBOT_IDS,
    WAYPOINTS,
    allocate_waypoints,
    validate_assignments,
)
from multi_robot_sim.health_monitor import stale_sensor_names
from multi_robot_sim.grid_path_planner import AStarGridPlanner


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RobotState:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.x = None
        self.y = None
        self.yaw = None
        self.front_clearance = None
        self.started_at = time.monotonic()
        self.last_odom_time = 0.0
        self.last_scan_time = 0.0
        self.goal_started = 0.0
        self.blocked_started = None
        self.queue = []
        self.current = None
        self.path = []
        self.path_index = 0
        self.failed = False
        self.publisher = None


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_stop_distance', 0.45)
        self.declare_parameter('obstacle_resume_distance', 0.60)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('blocked_timeout_sec', 8.0)
        self.declare_parameter('startup_grace_sec', 10.0)
        self.declare_parameter('sensor_timeout_sec', 2.0)

        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance').value)
        self.stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value)
        self.resume_distance = float(
            self.get_parameter('obstacle_resume_distance').value)
        if self.resume_distance <= self.stop_distance:
            raise ValueError('Resume distance must exceed stop distance.')
        self.goal_timeout = float(
            self.get_parameter('goal_timeout_sec').value)
        self.blocked_timeout = float(
            self.get_parameter('blocked_timeout_sec').value)
        self.startup_grace = float(
            self.get_parameter('startup_grace_sec').value)
        self.sensor_timeout = float(
            self.get_parameter('sensor_timeout_sec').value)
        if self.startup_grace < 0.0:
            raise ValueError('Startup grace cannot be negative.')
        if self.sensor_timeout <= 0.0:
            raise ValueError('Sensor timeout must be positive.')

        world_path = (
            Path(get_package_share_directory('multi_robot_sim'))
            / 'worlds' / 'inspection_world.world'
        )
        self.path_planner = AStarGridPlanner.from_sdf(
            world_path, resolution=0.10, inflation=0.22)

        self.robots = {}
        self.completed = set()
        self.unresolved = set()
        self.failed_robots = set()
        self.attempts = {}
        self.finished = False

        self.status_pub = self.create_publisher(
            String, '/mission/status', 10)
        self.assignment_pub = self.create_publisher(
            String, '/mission/assignments', 10)
        self.failure_sub = self.create_subscription(
            String, '/mission/failed_robot',
            self.failure_callback, 10)

        for robot_id in ROBOT_IDS:
            state = RobotState(robot_id)
            prefix = f'/{robot_id}'

            state.publisher = self.create_publisher(
                Twist, f'{prefix}/cmd_vel', 10)

            self.create_subscription(
                Odometry, f'{prefix}/odom',
                lambda msg, s=state: self.odom_callback(msg, s), 10)

            self.create_subscription(
                LaserScan, f'{prefix}/scan',
                lambda msg, s=state: self.scan_callback(msg, s), 10)

            self.robots[robot_id] = state

        self.assign_remaining()
        self.timer = self.create_timer(0.1, self.control_step)
        self.get_logger().info(
            'Mission executor started; waiting for robot odometry and scans.')

    def publish_status(self, event, robot_id=None, waypoint_id=None, detail=''):
        payload = {
            'event': event,
            'robot': robot_id,
            'waypoint': waypoint_id,
            'completed': sorted(self.completed),
            'failed_robots': sorted(self.failed_robots),
            'unresolved': sorted(self.unresolved),
            'detail': detail,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def stop_robot(self, state):
        state.publisher.publish(Twist())

    def stop_all(self):
        for state in self.robots.values():
            self.stop_robot(state)

    def odom_callback(self, msg, state):
        state.last_odom_time = time.monotonic()
        state.x = msg.pose.pose.position.x
        state.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        state.yaw = math.atan2(sin_yaw, cos_yaw)

    def scan_callback(self, msg, state):
        front = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = normalize_angle(
                msg.angle_min + i * msg.angle_increment)
            if abs(angle) <= math.radians(20):
                front.append(distance)

        state.front_clearance = min(front) if front else None
        state.last_scan_time = time.monotonic()

    def assign_remaining(self):
        """Allocate initial remaining work without including resolved goals."""
        excluded = self.completed | self.unresolved
        assignments = allocate_waypoints(
            waypoints=WAYPOINTS,
            robot_ids=ROBOT_IDS,
            completed=excluded,
            failed_robots=self.failed_robots,
        )
        validate_assignments(
            assignments, waypoints=WAYPOINTS, completed=excluded)

        for robot_id, state in self.robots.items():
            state.queue = list(assignments.get(robot_id, []))
            state.current = None
            state.path = []
            state.path_index = 0
            state.goal_started = 0.0
            state.blocked_started = None
            self.stop_robot(state)

        self.publish_assignment_snapshot()
        self.publish_status(
            'ASSIGNED',
            detail='Initial unique waypoint queues assigned to active robots.')

    def publish_assignment_snapshot(self):
        """Publish active assignments, including each robot's current goal."""
        payload = {}
        for robot_id, state in self.robots.items():
            if state.failed:
                continue
            work = ([] if state.current is None else [state.current])
            work.extend(state.queue)
            payload[robot_id] = [
                {'id': wp_id, 'x': x, 'y': y}
                for wp_id, x, y in work
            ]

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.assignment_pub.publish(msg)

    def failure_callback(self, msg, detail=None):
        """Mark one robot failed and transfer only its unfinished work."""
        robot_id = msg.data.strip()
        if robot_id not in self.robots:
            self.get_logger().error(
                f'Ignoring unknown failed robot ID: {robot_id}')
            return
        if robot_id in self.failed_robots:
            return

        state = self.robots[robot_id]
        current_id = state.current[0] if state.current else None
        excluded = self.completed | self.unresolved
        abandoned = []

        if state.current is not None and state.current[0] not in excluded:
            abandoned.append(state.current)

        seen = {wp[0] for wp in abandoned}
        for waypoint in state.queue:
            if waypoint[0] not in excluded and waypoint[0] not in seen:
                abandoned.append(waypoint)
                seen.add(waypoint[0])

        state.failed = True
        self.failed_robots.add(robot_id)
        self.stop_robot(state)
        state.current = None
        state.queue = []
        state.path = []
        state.path_index = 0
        state.goal_started = 0.0
        state.blocked_started = None

        self.publish_status(
            'ROBOT_FAILED', robot_id, current_id,
            detail or 'Robot stopped; only its unfinished work will be redistributed.')

        active_ids = [
            rid for rid in ROBOT_IDS if rid not in self.failed_robots
        ]
        if not active_ids:
            self.finished = True
            self.stop_all()
            self.publish_status(
                'MISSION_FAILED', detail='All robots unavailable.')
            return

        if abandoned:
            plan = allocate_waypoints(
                waypoints=tuple(abandoned),
                robot_ids=tuple(active_ids),
            )
            validate_assignments(plan, waypoints=tuple(abandoned))
            for rid, waypoints in plan.items():
                self.robots[rid].queue.extend(waypoints)

        self.publish_assignment_snapshot()
        self.publish_status(
            'REASSIGNED', robot_id,
            detail=f'Redistributed {len(abandoned)} unfinished waypoint(s).')

    def finish_waypoint(self, state, outcome, detail=''):
        """Record a goal result and retry a failure at most once."""
        if state.current is None:
            return

        waypoint = state.current
        waypoint_id = waypoint[0]
        self.stop_robot(state)

        if outcome == 'REACHED':
            self.completed.add(waypoint_id)
            self.unresolved.discard(waypoint_id)

        state.current = None
        state.path = []
        state.path_index = 0
        state.goal_started = 0.0
        state.blocked_started = None
        self.publish_status(outcome, state.robot_id, waypoint_id, detail)

        if outcome != 'REACHED':
            count = self.attempts.get(waypoint_id, 0) + 1
            self.attempts[waypoint_id] = count

            if count < len(ROBOT_IDS):
                active = [
                    candidate for candidate in self.robots.values()
                    if not candidate.failed
                ]
                alternatives = [
                    candidate for candidate in active
                    if candidate.robot_id != state.robot_id
                ]
                candidates = alternatives or active

                if candidates:
                    chosen = min(
                        candidates,
                        key=lambda candidate: (
                            len(candidate.queue), candidate.robot_id)
                    )
                    chosen.queue.append(waypoint)
                    self.publish_status(
                        'WAYPOINT_REQUEUED', chosen.robot_id, waypoint_id,
                        f'Retry {count + 1} scheduled after {outcome}.')
                else:
                    self.unresolved.add(waypoint_id)
            else:
                self.unresolved.add(waypoint_id)
                self.publish_status(
                    'WAYPOINT_UNRESOLVED', state.robot_id, waypoint_id,
                    'Attempt limit reached; this goal will not be retried.')

        self.publish_assignment_snapshot()

    def control_robot(self, state, now):
        if state.failed:
            self.stop_robot(state)
            return

        if state.x is None or state.yaw is None:
            self.stop_robot(state)
            return

        # Require a recent scan before commanding forward motion.
        if now - state.last_scan_time > self.sensor_timeout:
            self.stop_robot(state)
            return

        if state.current is None:
            while state.queue and state.queue[0][0] in self.completed:
                state.queue.pop(0)
            if not state.queue:
                self.stop_robot(state)
                return

            state.current = state.queue.pop(0)
            state.goal_started = now
            state.blocked_started = None
            try:
                state.path = self.path_planner.plan(
                    (state.x, state.y),
                    (state.current[1], state.current[2]),
                )
            except ValueError as exc:
                self.finish_waypoint(
                    state, 'NO_PATH',
                    f'A* rejected the start or goal: {exc}')
                return
            if state.path is None:
                self.finish_waypoint(
                    state, 'NO_PATH',
                    'A* could not find a route through the planning grid.')
                return
            state.path_index = 1 if len(state.path) > 1 else 0
            self.publish_status(
                'GOAL_STARTED', state.robot_id, state.current[0],
                f'target=({state.current[1]:.2f},{state.current[2]:.2f})')

        waypoint_id, target_x, target_y = state.current

        if now - state.goal_started > self.goal_timeout:
            self.finish_waypoint(
                state, 'TIMEOUT', 'Goal exceeded configured time limit.')
            return

        if state.blocked_started is not None:
            clearance = state.front_clearance
            if clearance is None or clearance < self.resume_distance:
                self.stop_robot(state)
                if now - state.blocked_started >= self.blocked_timeout:
                    self.finish_waypoint(
                        state, 'BLOCKED', 'Obstacle remained in front.')
                return
            self.publish_status(
                'OBSTACLE_CLEARED', state.robot_id, waypoint_id,
                f'front clearance={clearance:.2f} m')
            state.blocked_started = None
        elif (state.front_clearance is not None
              and state.front_clearance < self.stop_distance):
            self.stop_robot(state)
            state.blocked_started = now
            self.publish_status(
                'OBSTACLE_STOP', state.robot_id, waypoint_id,
                f'front clearance={state.front_clearance:.2f} m')
            return

        if not state.path:
            self.finish_waypoint(
                state, 'NO_PATH', 'No A* path is available.')
            return

        path_x, path_y = state.path[state.path_index]
        dx = path_x - state.x
        dy = path_y - state.y
        distance = math.hypot(dx, dy)
        final_path_point = state.path_index == len(state.path) - 1
        path_tolerance = 0.05 if final_path_point else 0.12

        if distance <= path_tolerance:
            if not final_path_point:
                state.path_index += 1
                self.stop_robot(state)
                return

            goal_distance = math.hypot(
                target_x - state.x, target_y - state.y)
            if goal_distance <= self.goal_tolerance:
                self.finish_waypoint(
                    state, 'REACHED',
                    f'A* path followed; goal distance={goal_distance:.3f} m; '
                    f'position=({state.x:.3f},{state.y:.3f})')
                return

            # Correct the final sub-cell error only after following the path.
            dx = target_x - state.x
            dy = target_y - state.y
            distance = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - state.yaw)

        cmd = Twist()
        cmd.angular.z = clamp(1.5 * yaw_error, -0.8, 0.8)
        if abs(yaw_error) < 0.45:
            cmd.linear.x = min(0.08, 0.5 * distance)
            cmd.linear.x = max(0.025, cmd.linear.x)

        state.publisher.publish(cmd)

    def detect_robot_failures(self, now):
        """Automatically fail robots whose sensor streams go stale."""
        for state in self.robots.values():
            if state.failed:
                continue

            stale = stale_sensor_names(
                now=now,
                started_at=state.started_at,
                last_odom_time=state.last_odom_time,
                last_scan_time=state.last_scan_time,
                startup_grace_sec=self.startup_grace,
                sensor_timeout_sec=self.sensor_timeout,
            )
            if not stale:
                continue

            detail = (
                'Sensor heartbeat timeout: stale '
                + ', '.join(stale)
                + ' stream(s).'
            )
            self.get_logger().error(
                f'{state.robot_id} declared failed: {detail}')
            message = String()
            message.data = state.robot_id
            self.failure_callback(message, detail=detail)

    def control_step(self):
        if self.finished:
            self.stop_all()
            return

        now = time.monotonic()
        self.detect_robot_failures(now)
        if self.finished:
            self.stop_all()
            return

        for state in self.robots.values():
            self.control_robot(state, now)

        if len(self.completed) == len(WAYPOINTS):
            self.finished = True
            self.stop_all()
            self.publish_status(
                'MISSION_COMPLETE',
                detail='All ten unique waypoints reached.')
            return

        all_work_done = all(
            state.current is None and not state.queue
            for state in self.robots.values()
        )
        if all_work_done:
            remaining = {wp[0] for wp in WAYPOINTS} - self.completed
            self.unresolved.update(remaining)
            self.finished = True
            self.stop_all()
            self.publish_status(
                'MISSION_INCOMPLETE',
                detail=f'No work remains; unresolved={sorted(remaining)}.')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received; stopping robots.')
    finally:
        if rclpy.ok():
            node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
