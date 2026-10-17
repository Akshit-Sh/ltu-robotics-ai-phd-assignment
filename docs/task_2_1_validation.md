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


## Follow-up validation — 17 October 2026

- The earlier obstacle/retry notes above describe the baseline controller before A* path planning was integrated.
- The updated executor uses a custom A* grid derived from static collision geometry in the Gazebo SDF world, with 0.10 m resolution and 0.22 m obstacle inflation.
- The live A*-enabled Gazebo run reached all ten unique waypoints and logged `MISSION_COMPLETE`, with no unresolved waypoints or failed robots in that run. Final logged goal distances were approximately 0.048–0.050 m.
- A separate route test from (1.4, -3.4) to (1.4, -1.6) found a four-point detour around `block_b`, with 0.70 m maximum lateral deviation from the direct line.
- This is static-world A* planning, not Nav2. The planner does not use the saved SLAM map; dynamic obstacle rerouting and guaranteed collision avoidance have not been established.
- At the 17 October follow-up, the isolated package build/test result was 52 tests collected, 51 passed, 1 skipped, 0 failed.


## Regression follow-up — 18 October 2026

- Added a focused LLM-planner regression test for an empty Ollama task list; the empty plan is rejected without invoking the deterministic fallback.
- Latest package test run: 53 tests collected, 52 passed, 1 skipped, 0 failed. Two lint-tool deprecation warnings did not fail the run.

## Corrected-world integration validation — 18 October 2026

- Corrected `multi_robot.launch.py` to load the package's `worlds/inspection_world.world` instead of TurtleBot3's `empty_world.world`. Rebuilt the package with `colcon build --packages-select multi_robot_sim --symlink-install`. Gazebo was relaunched, and the walls and obstacles were visually confirmed.
- Started `mission_executor` separately after Gazebo. The executor published `ASSIGNED` with unique initial waypoint queues and began execution after receiving robot odometry and LiDAR data.
- The A*-enabled run reached all ten unique waypoints, `wp01` through `wp10`. The final `MISSION_COMPLETE` status listed all ten IDs in `completed`, with empty `unresolved` and `failed_robots` fields.
- The ten logged goal distances were approximately 0.048–0.050 m, within the configured 0.15 m goal tolerance.
- The run logged `OBSTACLE_STOP` and `OBSTACLE_CLEARED` events during waypoint execution. Examples included r2 at wp03 (0.44 m clearance, later 2.40 m), r0 at wp04 (0.44 m, later 0.60 m), and r0 at wp07 (0.26 m and 0.24 m stops, with subsequent clearance events).
- This verifies the static-world A* planner, odometry-based path following, and forward LiDAR stop/resume guard in the corrected inspection world for this run. It does not establish dynamic-obstacle rerouting, ground-truth map accuracy, Nav2 operation, or guaranteed collision avoidance.
- Evidence source: terminal output from the interactive Gazebo/executor run on 18 October 2026. This summary records the observed output; it is not a separately archived raw log file.
