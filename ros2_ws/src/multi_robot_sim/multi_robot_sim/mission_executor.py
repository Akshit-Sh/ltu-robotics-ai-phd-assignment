"""Centralized three-robot waypoint mission executor."""

import json
import math
import time

import rclpy
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
        self.last_scan_time = 0.0
        self.goal_started = 0.0
        self.blocked_started = None
        self.queue = []
        self.current = None
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

    def failure_callback(self, msg):
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
        state.goal_started = 0.0
        state.blocked_started = None

        self.publish_status(
            'ROBOT_FAILED', robot_id, current_id,
            'Robot stopped; only its unfinished work will be redistributed.')

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
        if now - state.last_scan_time > 2.0:
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

        dx = target_x - state.x
        dy = target_y - state.y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.finish_waypoint(
                state, 'REACHED',
                f'distance={distance:.3f} m; '
                f'position=({state.x:.3f},{state.y:.3f})')
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - state.yaw)

        cmd = Twist()
        cmd.angular.z = clamp(1.5 * yaw_error, -0.8, 0.8)
        if abs(yaw_error) < 0.45:
            cmd.linear.x = min(0.08, 0.5 * distance)
            cmd.linear.x = max(0.025, cmd.linear.x)

        state.publisher.publish(cmd)

    def control_step(self):
        if self.finished:
            self.stop_all()
            return

        now = time.monotonic()
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
