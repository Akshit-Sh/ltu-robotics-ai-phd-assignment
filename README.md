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
| SLAM / mapping | Pending |
| Nav2 navigation | Pending |
| Centralized ten-waypoint planner | Pending |
| Semantic mission planning and event-driven replanning | Pending |

Statuses reflect work completed and tested so far; pending components are not represented as implemented.

## Technology Stack

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 Burger simulation model
- `robot_state_publisher`
- Xacro and URDF
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

The robots publish transforms on the shared `/tf` and `/tf_static` topics. Frame IDs use robot-specific prefixes, such as `r0/base_link`, `r1/base_link`, and `r2/base_link`.

## Verification Performed

The following checks have been completed in the development environment:

1. Confirmed that all three robots spawn in Gazebo.
2. Confirmed that each robot publishes its own command, LiDAR, IMU, and odometry topics.
3. Commanded `r0` and observed its odometry change while `r1` and `r2` remained at their initial positions during that test.
4. Verified `odom -> base_footprint` transforms for all three robots.
5. Verified static link transforms, including `base_link`, `imu_link`, `base_scan`, and `caster_back_link`, for all three robots.
6. Measured LiDAR publication rates during one Gazebo/WSL session.

### Observed LiDAR Rates

| Topic | Approximate observed rate |
|---|---:|
| `/r0/scan` | 4.0 Hz |
| `/r1/scan` | 4.0 Hz |
| `/r2/scan` | 3.4 Hz |

These are observed rates from the current simulation session, not guaranteed or configured target rates. Further performance investigation is pending.

## Current Limitations

- The current three-robot simulation uses TurtleBot3 Burger models.
- A complete SLAM and mapping workflow has not yet been implemented or verified.
- Nav2 has not yet been configured and tested for all three robots.
- The centralized planner for ten unique waypoints has not yet been implemented.
- Semantic scene-graph grounding, plan validation, scripted mission events, and event-driven replanning remain pending.
- The current Burger simulation does not yet provide the RGB-D camera required by the assignment specification.
- The current work is a simulation-based implementation; no physical robot deployment is claimed.

## Development Plan

The next stages are:

1. Investigate simulation timing and sensor publication rates.
2. Implement and evaluate mapping.
3. Configure and test Nav2 independently for each robot.
4. Implement centralized assignment of ten unique waypoints without duplicate visits.
5. Add robot-failure handling and evaluate replanning.
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
