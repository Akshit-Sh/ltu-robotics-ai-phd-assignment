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

## Robot-failure recovery
- Injected mission-level failure for r0 through /mission/failed_robot.
- The executor emitted ROBOT_FAILED and redistributed four unfinished goals.
- r1 and r2 continued executing the reassigned mission.
- The observed final completed set contained wp01 through wp10.
- This was a software-level availability test, not physical hardware failure.

## Mapping results
- Ran SLAM Toolbox while the mission executed in the bounded inspection world.
- Saved map image and metadata: maps/inspection_map.pgm and
  maps/inspection_map.yaml.
- Grid dimensions: 152 x 160 cells; resolution: 0.05 m/cell.
- Physical extent: 7.60 x 8.00 m; total cells: 24,320.
- Free: 14,616 cells (60.1%).
- Occupied: 830 cells (3.4%).
- Unknown: 8,874 cells (36.5%).
- Known-cell coverage: 63.5%.
- Cell statistics show partial map coverage; geometric accuracy has not been
  validated against a ground-truth map.

## Remaining limitation
- The controller uses odometry-based motion and a forward LiDAR stop guard,
  not global path planning or guaranteed collision avoidance.
