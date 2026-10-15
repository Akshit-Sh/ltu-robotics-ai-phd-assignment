"""Basic odometry-based waypoint navigation with a LiDAR stop guard."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('target_x', 1.4)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('robot_id', 'r0')
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('obstacle_stop_distance', 0.45)

        self.robot_id = self.get_parameter('robot_id').value
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance').value)
        self.stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value)

        prefix = f'/{self.robot_id}'
        self.cmd_pub = self.create_publisher(Twist, f'{prefix}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, f'{prefix}/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, f'{prefix}/scan', self.scan_callback, 10)

        self.x = None
        self.y = None
        self.yaw = None
        self.front_clearance = None
        self.finished = False
        self.blocked = False
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_step)
        self.get_logger().info(
            f'{self.robot_id}: navigating toward '
            f'({self.target_x:.2f}, {self.target_y:.2f})')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(sin_yaw, cos_yaw)

    def scan_callback(self, msg):
        front = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = normalize_angle(angle)
            if abs(angle) <= math.radians(20):
                front.append(distance)

        self.front_clearance = min(front) if front else None

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_step(self):
        if self.finished:
            self.stop_robot()
            return

        if self.x is None or self.yaw is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > 45.0:
            self.finished = True
            self.stop_robot()
            self.get_logger().error('TIMEOUT: waypoint not reached.')
            return

        if (self.front_clearance is not None
                and self.front_clearance < self.stop_distance):
            self.stop_robot()
            if not self.blocked:
                self.get_logger().error(
                    f'OBSTACLE STOP: front clearance '
                    f'{self.front_clearance:.2f} m')
            self.blocked = True
            return

        self.blocked = False

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.finished = True
            self.stop_robot()
            self.get_logger().info(
                f'REACHED: distance to goal {distance:.3f} m; '
                f'final position ({self.x:.3f}, {self.y:.3f})')
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - self.yaw)

        cmd = Twist()
        cmd.angular.z = clamp(1.5 * yaw_error, -0.8, 0.8)

        if abs(yaw_error) < 0.45:
            cmd.linear.x = min(0.08, 0.5 * distance)
            cmd.linear.x = max(0.025, cmd.linear.x)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
