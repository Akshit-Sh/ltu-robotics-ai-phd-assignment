# Design Decisions

## 1. Development Environment

The simulation is developed in Ubuntu 22.04 under WSL2, using ROS 2
Humble and Gazebo Classic. Software rendering is enabled when needed
to support Gazebo GUI execution in this environment.

## 2. Simulation Platform

TurtleBot3 Burger models are used as the initial robot platform.
Three robots are spawned in a shared Gazebo world to establish a
baseline for multi-robot simulation and communication.

Initial spawn positions:

| Robot | x (m) | y (m) |
|---|---:|---:|
| r0 | 0.0 | 0.0 |
| r1 | 2.0 | 0.0 |
| r2 | 0.0 | 2.0 |

These positions are initial conditions, not assigned mission goals.

## 3. Robot Namespaces

Each robot uses a separate ROS 2 namespace: `r0`, `r1`, and `r2`.
Robot-specific topics include `/rX/cmd_vel`, `/rX/scan`,
`/rX/imu`, and `/rX/odom`.

This arrangement separates robot commands and sensor streams and
makes it possible to address each robot independently.

An isolation test commanded `r0` and observed its odometry change
while `r1` and `r2` remained at their initial positions during that
test. This is evidence for that test case, not a claim that every
possible interaction has been exhaustively tested.

## 4. TF Frame Design

Each robot has a `robot_state_publisher` using a URDF expanded with
a robot-specific frame prefix. Examples include `r0/base_link`,
`r1/base_link`, and `r2/base_link`.

The robots publish transforms on the shared `/tf` and `/tf_static`
topics. The verified dynamic transform chains include:

- `r0/odom` -> `r0/base_footprint`
- `r1/odom` -> `r1/base_footprint`
- `r2/odom` -> `r2/base_footprint`

Static transforms for the robot links, including the base, IMU,
LiDAR, and caster links, were also verified for all three robots.

The purpose of the prefixes is to avoid collisions between robot
frame IDs in the shared TF system.

## 5. Sensor Rate Measurements

LiDAR messages were observed on `/r0/scan`, `/r1/scan`, and
`/r2/scan`. During one Gazebo/WSL session, approximate observed
publication rates were:

| Topic | Observed rate |
|---|---:|
| `/r0/scan` | 4.0 Hz |
| `/r1/scan` | 4.0 Hz |
| `/r2/scan` | 3.4 Hz |

These are session-specific measurements, not configured targets or
guaranteed rates. The difference between the observed rates and the
expected sensor performance requires further investigation.

## 6. Current Scope and Limitations

The current baseline verifies robot spawning, namespaced command and
sensor topics, odometry, and robot TF frames.

The following are not yet considered implemented or verified:

- RGB-D camera integration for every robot.
- A complete mapping workflow.
- Nav2 configuration and navigation tests for every robot.
- Centralized allocation and execution of ten unique waypoints.
- Robot-failure detection and recovery.
- Semantic scene-graph grounding and plan validation.
- Event-driven mission replanning.

These items must be implemented and tested before the corresponding
assignment requirements can be marked complete.

## 7. Reproducibility Principle

Measured results, tested behavior, design intentions, and planned
features should be reported separately. A feature is not considered
verified solely because its software package is installed or because
its implementation is planned.
