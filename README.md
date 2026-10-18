
# Multi-Robot Robotics and AI Assignment

**LTU Robotics and AI PhD Assignment — Luleå University of Technology**

This repository contains the simulation environment, ROS 2 package, experiments, documentation, and report materials for a multi-robot robotics and AI assignment.

The project is being developed incrementally, with an emphasis on reproducible experiments, independent robot namespaces, verified TF frames, and clearly documented limitations.

## Project Status

| Component | Status |
|---|---|
| ROS 2 Humble workspace | Set up |
| Gazebo Classic simulation | Three robots spawned |
| Robot namespaces (`r0`, `r1`, `r2`) | Verified |
| Independent command-topic isolation | Tested by commanding `r0` |
| Dynamic odometry TF | Verified for all three robots |
| Static robot-link TF | Verified for all three robots |
| LiDAR topic publication | Verified for all three robots |
| RGB-D camera topics | Verified for all three robots |
| Camera frame IDs | Checked for `r0` RGB and depth images |
| Base-to-camera static TF | Verified for all three robots |
| SLAM / mapping | Executed; map saved |
| Custom A* static-world planner | Implemented and tested; uses Gazebo SDF geometry, not the SLAM map |
| Nav2 global planning | Not configured or validated |
| Ten-waypoint mission | All 10 waypoints reached in the A*-enabled Gazebo run |
| Semantic planning baseline and event-driven replanning | Implemented; 58 tests collected: 57 passed, 1 skipped, 0 failed |

Statuses reflect work completed and tested so far; pending components are not represented as implemented.

## Technology Stack

- Ubuntu 22.04 under WSL2
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 Burger simulation model
- `robot_state_publisher`
- Xacro and URDF
- `tf2_ros`
- Gazebo ROS camera plugin
- Nav2 and `slam_toolbox` installed for planned development
- Python and Git

## Repository Structure

```text
ltu-robotics-ai-phd-assignment/
├── README.md
├── .gitignore
├── docs/
│   ├── design_decisions.md
│   └── time_log.md
├── report/
│   ├── report.md
│   ├── references.bib
│   └── assignment_report.pdf
└── ros2_ws/
    └── src/
        └── multi_robot_sim/
            ├── launch/
            │   ├── multi_robot.launch.py
            │   └── single_robot.launch.py
            ├── models/
            │   └── turtlebot3_burger/
            ├── multi_robot_sim/
            ├── package.xml
            ├── resource/
            ├── setup.cfg
            └── setup.py
```

## Prerequisites

The current simulation was developed in Ubuntu 22.04 under WSL2, using ROS 2 Humble and Gazebo Classic.

The ROS packages used by the simulation include:

- `gazebo_ros_pkgs`
- `turtlebot3_gazebo`
- `turtlebot3_description`
- `navigation2`
- `nav2_bringup`
- `slam_toolbox`

Install or verify the required packages in an appropriately configured ROS 2 Humble environment before running the project.

## Build

Open a terminal and run:

```bash
source /opt/ros/humble/setup.bash

cd ~/ltu_rai_assignment/ros2_ws
colcon build --packages-select multi_robot_sim --symlink-install

source install/setup.bash
```

## Run the Multi-Robot Simulation

Run the following in the terminal where you want to launch Gazebo:

```bash
source /opt/ros/humble/setup.bash
source ~/ltu_rai_assignment/ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch multi_robot_sim multi_robot.launch.py
```

The software-rendering setting is useful for the WSL2 environment used during development. Other environments may not require it.

The current launch file starts one Gazebo server and GUI and spawns three robots at the following initial positions:

| Robot | Initial x | Initial y |
|---|---:|---:|
| `r0` | 0.0 m | 0.0 m |
| `r1` | 2.0 m | 0.0 m |
| `r2` | 0.0 m | 2.0 m |

## Task 2.1 — Mission reproduction

Launch Gazebo using the command above. In a second sourced terminal run `ros2 run multi_robot_sim mission_executor`. Monitor `/mission/status` and wait for `MISSION_COMPLETE`; keep this successful run free of injected failures.

For a separate recovery run, publish `r0` to `/mission/failed_robot` as `std_msgs/msg/String` using `ros2 topic pub --once /mission/failed_robot std_msgs/msg/String "{data: r0}"`. The number of unfinished goals transferred depends on injection timing; the recorded run transferred four. This is a software-level failure simulation, not physical hardware failure.

## Namespaces and Topics

Each robot has its own namespaced command, sensor, and odometry topics.

| Purpose | `r0` | `r1` | `r2` |
|---|---|---|---|
| Velocity command | `/r0/cmd_vel` | `/r1/cmd_vel` | `/r2/cmd_vel` |
| LiDAR scan | `/r0/scan` | `/r1/scan` | `/r2/scan` |
| IMU | `/r0/imu` | `/r1/imu` | `/r2/imu` |
| Odometry | `/r0/odom` | `/r1/odom` | `/r2/odom` |
| RGB image | `/r0/camera/color/image_raw` | `/r1/camera/color/image_raw` | `/r2/camera/color/image_raw` |
| Depth image | `/r0/camera/depth/image_raw` | `/r1/camera/depth/image_raw` | `/r2/camera/depth/image_raw` |
| RGB camera info | `/r0/camera/color/camera_info` | `/r1/camera/color/camera_info` | `/r2/camera/color/camera_info` |
| Depth camera info | `/r0/camera/depth/camera_info` | `/r1/camera/depth/camera_info` | `/r2/camera/depth/camera_info` |
| Depth point cloud | `/r0/camera/depth/points` | `/r1/camera/depth/points` | `/r2/camera/depth/points` |

The robots publish transforms on the shared `/tf` and `/tf_static` topics. Frame IDs use robot-specific prefixes, such as `r0/base_link`, `r1/base_link`, and `r2/base_link`.

## RGB-D Camera Configuration

Each robot model contains a simulated RGB-D camera configured with:

| Parameter | Value |
|---|---|
| Image resolution | 320 × 240 pixels |
| Configured update rate | 10 Hz |
| Horizontal field of view | 1.047 rad |
| Near clipping distance | 0.10 m |
| Far clipping distance | 8.0 m |
| Camera frame | `rX/camera_link` |

The RGB and depth image headers checked for `r0` reported `r0/camera_link`. Static transforms from each robot's `base_link` to `camera_link` were added to the launch file. Transform lookups from `odom` to `camera_link` returned valid results for all three robots.

These checks verify that the frames and image streams are available. They do not, by themselves, prove that the configured camera pose is perfectly aligned with the simulated camera geometry.

## Verification Performed

The following checks have been completed in the development environment:

1. Confirmed that all three robots spawn in Gazebo.
2. Confirmed namespaced command, LiDAR, IMU, and odometry topics.
3. Commanded `r0` and observed its odometry change while `r1` and `r2` remained at their initial positions during that test.
4. Verified `odom -> base_footprint` transforms for all three robots.
5. Verified static robot-link transforms, including the base, IMU, LiDAR, and caster links, for all three robots.
6. Discovered RGB image, depth image, camera-info, and depth point-cloud topics for all three robots.
7. Checked RGB and depth image frame IDs for `r0`.
8. Verified `base_link -> camera_link` and `odom -> camera_link` transform lookups for all three robots.
9. Measured LiDAR and camera publication rates using short ROS CLI checks.

### Preliminary Observed Sensor Rates

The LiDAR and cameras are configured for 10 Hz. The LiDAR measurements in the table below are earlier preliminary results from before the LiDAR update; a later run measured approximately 8.7 Hz. Actual rates vary with Gazebo/WSL simulation load.

| Sensor stream | Preliminary observed rate |
|---|---:|
| `/r0/scan` | Approximately 3.2–4.0 Hz across sessions |
| `/r1/scan` | Approximately 3.2–4.0 Hz across sessions |
| `/r2/scan` | Approximately 3.4–3.6 Hz across sessions |
| `r0` RGB | Approximately 6.7 Hz |
| `r1` RGB | Approximately 7.0 Hz |
| `r2` RGB | Stable RGB measurement not obtained |
| RGB-D depth streams | Approximately 6.5–7.2 Hz across available checks |

These values are preliminary observations, not guaranteed performance figures. Measurements used short-lived ROS CLI processes, and some checks showed startup or ROS context shutdown warnings. Longer-duration measurements under controlled conditions are needed for reliable benchmarking.

## Task 2.2 — Language-Grounded Semantic Planning

### Scene graph and grounding

- `config/semantic/scene_graph.json` defines the configured robots, inspection locations, obstacles, and relations. This is configured world knowledge, not an automatically perceived scene graph.
- `semantic_planner.py` provides the deterministic natural-language planning baseline.
- `llm_semantic_planner.py` integrates the locally hosted Qwen 2.5 1.5B model through the Ollama HTTP API. It requests a structured JSON plan and grounds targets against configured location IDs.
- The model-backed planner normalises supported action names, maps accepted location aliases to configured IDs, and checks plans using `semantic_validator.py`.
- The validator rejects malformed plans, unsupported actions, unknown targets or robots, unavailable robots, and duplicate task IDs.

### Validation and event-driven replanning

- An end-to-end test successfully generated and validated an inspection task targeting `inspection_east` with robot `r0`.
- Invalid-target rejection was verified directly: a plan targeting `made_up_room` returned `valid: false` with the error `unknown target 'made_up_room'` and was rejected before execution.
- `semantic_events.py` demonstrates two scripted events: a robot becoming unavailable and an inspection target becoming unavailable. The resulting plans are validated against the updated state.
- The event demonstration currently uses the deterministic planner. It is not evidence that Qwen regenerates plans after every event.
- Semantic planning and event handling are software-level demonstrations and are not connected to physical robot execution.

### Model and reproducibility

- Model: `qwen2.5:1.5b`, run locally with Ollama.
- The prompt requests JSON containing a `tasks` list with `task_id`, `action`, `target`, and robot assignment. It restricts actions and entity IDs to the configured scene graph and asks for an empty task list when the request is ambiguous or unsupported.
- The model-backed command is `llm_semantic_planner`.

After building and sourcing the workspace, reproduce the demonstrations:

```bash
ros2 run multi_robot_sim semantic_planner
ros2 run multi_robot_sim llm_semantic_planner
ros2 run multi_robot_sim semantic_events
colcon test --packages-select multi_robot_sim --event-handlers console_direct+
```

The latest regression run collected 58 tests: 57 passed, 1 skipped, 0 failed. Python lint tooling emitted deprecation warnings that did not fail the tests.
The model-backed planner has been tested on a representative unambiguous mission. Broader evaluation of ambiguous requests, model failure, grounding accuracy, and performance against the deterministic baseline remains pending.

## Current Limitations

- The simulation uses TurtleBot3 Burger models.
- Sensor publication rates are below their configured targets in the observed sessions; the cause has not yet been isolated.
- SLAM Toolbox was run and a map was saved; ground-truth map accuracy has not been evaluated.
- Full Nav2 global planning has not been configured and validated for all three robots.
- The ten-waypoint mission and centralized execution were tested in simulation.
- Software-level failure-event handling and reassignment were tested; physical failure modes remain untested.
- Deterministic semantic planning, a locally hosted Qwen 2.5 1.5B planner, and scripted event replanning are implemented. Model-backed planning has been tested on a representative mission; broader evaluation remains pending.
- Long-duration sensor-rate and simulation-performance benchmarks remain pending.
- The current work is simulation-based; no physical robot deployment is claimed.

## Development Plan

The next stages are:

1. Investigate simulation timing and sensor publication rates.
2. Improve mapping coverage and evaluate the saved map against ground truth.
3. Configure and test Nav2 independently for each robot.
4. Keep the verified A*-enabled waypoint mission and obstacle-detour regression test reproducible.
5. Extend failure-recovery testing to additional failure scenarios.
6. Extend the implemented semantic planning baseline, scene-graph grounding, plan validation, and scripted event replanning; evaluate broader language coverage and model failure cases.
7. Document long-term inspection autonomy challenges, the research problem, and the literature review.
8. Complete reproducibility instructions, test evidence, and the final assignment report.

## Documentation

- [`docs/design_decisions.md`](docs/design_decisions.md) — design decisions and rationale.
- [`docs/time_log.md`](docs/time_log.md) — development milestones and test observations.
- [`report/report.md`](report/report.md) — report draft.
- [`report/references.bib`](report/references.bib) — bibliography.

## Reproducibility and Reporting

Commands, test observations, limitations, and development decisions should be documented as the project evolves. Results should distinguish measured observations from expected behavior, and all assignment requirements should be verified before being marked complete.

## License

See the repository's `LICENSE` file for the applicable terms.
