#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MotionTest(Node):
    def __init__(self):
        super().__init__("r0_motion_test")

        self.publisher = self.create_publisher(
            Twist, "/r0/cmd_vel", 10
        )
        self.subscription = self.create_subscription(
            Odometry, "/r0/odom", self.odom_callback, 10
        )

        self.start_x = None
        self.current_x = None
        self.samples = 0
        self.phase = "waiting"
        self.phase_start = time.monotonic()

        self.timer = self.create_timer(0.1, self.control)
        self.get_logger().info("Waiting for odometry and command connection.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.samples += 1

        if self.start_x is None:
            self.start_x = self.current_x

    def publish_velocity(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.publisher.publish(msg)

    def control(self):
        elapsed = time.monotonic() - self.phase_start

        if self.phase == "waiting":
            self.publish_velocity(0.0)

            if elapsed >= 2.0 and self.current_x is not None:
                self.start_x = self.current_x
                self.samples = 0
                self.phase = "moving"
                self.phase_start = time.monotonic()
                self.get_logger().info(
                    f"Starting position x={self.start_x:.4f} m; "
                    "moving at 0.05 m/s for 6 seconds."
                )

        elif self.phase == "moving":
            self.publish_velocity(0.05)

            if elapsed >= 6.0:
                self.phase = "stopping"
                self.phase_start = time.monotonic()
                self.publish_velocity(0.0)
                self.get_logger().info(
                    "Movement phase complete; publishing zero velocity."
                )

        elif self.phase == "stopping":
            self.publish_velocity(0.0)

            if elapsed >= 3.0:
                end_x = self.current_x
                displacement = end_x - self.start_x

                self.get_logger().info(
                    f"Start x={self.start_x:.4f} m; "
                    f"final x={end_x:.4f} m; "
                    f"signed displacement={displacement:.4f} m; "
                    f"odometry samples={self.samples}."
                )
                self.get_logger().info(
                    "Test complete. Robot is receiving zero velocity."
                )
                self.phase = "done"

        elif self.phase == "done":
            self.publish_velocity(0.0)


def main():
    rclpy.init()
    node = MotionTest()

    try:
        while rclpy.ok() and node.phase != "done":
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Continue sending stop commands before shutting down.
        for _ in range(20):
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
