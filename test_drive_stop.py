#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = rclpy.create_node('drive_stop_test')
    pub = node.create_publisher(Twist, '/r0/cmd_vel', 10)

    # Allow the publisher to connect before commanding motion.
    print('Waiting for the command subscriber...', flush=True)
    deadline = time.monotonic() + 10.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        print('ERROR: No subscriber on /r0/cmd_vel.', flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return

    print('Sending 0.03 m/s forward for 1 second.', flush=True)
    move = Twist()
    move.linear.x = 0.03
    start = time.monotonic()
    while time.monotonic() - start < 1.0:
        pub.publish(move)
        rclpy.spin_once(node, timeout_sec=0.05)

    print('Sending ZERO velocity continuously for 5 seconds.', flush=True)
    stop = Twist()
    start = time.monotonic()
    while time.monotonic() - start < 5.0:
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    print('Stop phase complete; publishing zero once more.', flush=True)
    pub.publish(stop)
    time.sleep(0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
