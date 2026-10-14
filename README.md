
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
| SLAM / mapping | Pending |
| Nav2 navigation | Pending |
| Centralized ten-waypoint planner | Pending |
| Semantic mission planning and event-driven replanning | Pending |

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
│   └── references.bib
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
colcon build --packages-select multi_robot_sim

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

The LiDAR is configured for 5 Hz and the cameras for 10 Hz. The observed rates were lower in the current Gazebo/WSL environment.

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

## Current Limitations

- The simulation uses TurtleBot3 Burger models.
- Sensor publication rates are below their configured targets in the observed sessions; the cause has not yet been isolated.
- A complete SLAM and mapping workflow has not yet been implemented or verified.
- Nav2 has not yet been configured and tested for all three robots.
- The centralized planner for ten unique waypoints has not yet been implemented.
- Robot-failure detection and recovery remain pending.
- Semantic scene-graph grounding, plan validation, scripted mission events, and event-driven replanning remain pending.
- Long-duration sensor-rate and simulation-performance benchmarks remain pending.
- The current work is simulation-based; no physical robot deployment is claimed.

## Development Plan

The next stages are:

1. Investigate simulation timing and sensor publication rates.
2. Implement and evaluate mapping.
3. Configure and test Nav2 independently for each robot.
4. Implement centralized assignment of ten unique waypoints without duplicate visits.
5. Add robot-failure handling and evaluate recovery.
6. Develop semantic mission planning, scene-graph grounding, plan validation, and event-driven replanning.
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
