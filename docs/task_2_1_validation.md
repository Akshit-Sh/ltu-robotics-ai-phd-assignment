# Task 2.1 — Simulation Validation

Date: 2026-10-16
Environment: ROS 2 Humble, Ubuntu 22.04, Gazebo Classic (headless)

## Multi-robot setup
- Three TurtleBot3 Burger robots: r0, r1, r2.
- Initial spawn positions: r0=(0,0), r1=(2,0), r2=(0,2).
- Verified per-robot command velocity, odometry, and LiDAR scan topics.
- RGB-D camera and IMU topics were verified in earlier sensor checks.

## Centralized waypoint mission
- Ten unique waypoints: wp01 through wp10.
- Goal tolerance: 0.15 m.
- Observed terminal event: MISSION_COMPLETE.
- Final completed set contained all ten unique waypoint IDs.
- Final unresolved list was empty.
- Final failed-robot list was empty.
- Logged goal errors were approximately 0.143–0.150 m.

## Obstacle and retry behaviour
- The controller stopped for close forward LiDAR returns.
- Hysteresis uses a 0.45 m stop threshold and 0.60 m resume threshold.
- Blocked goals were requeued and subsequently reached in the successful run.
- This is a reactive stopping guard and retry mechanism, not global path planning
  or a guarantee of collision-free motion.

## Shutdown and automated checks
- Ctrl+C logged that robots were being stopped, with no shutdown traceback.
- Latest package test summary: 8 tests, 0 errors, 0 failures, 1 skipped.

## Remaining validation
- Deliberately fail one robot and verify reassignment of its unfinished work.
- Improve and validate mapping evidence; the earlier SLAM map was sparse and
  mostly unknown.
- Preserve run logs and document limitations in the final report.
