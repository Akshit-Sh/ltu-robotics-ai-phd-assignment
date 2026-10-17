
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

RGB-D camera topics are also namespaced. They include:

- `/rX/camera/color/image_raw`
- `/rX/camera/color/camera_info`
- `/rX/camera/depth/image_raw`
- `/rX/camera/depth/camera_info`
- `/rX/camera/depth/points`

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
LiDAR, and caster links, were verified for all three robots.

Each robot's RGB-D camera publishes images with a frame ID following
the pattern `rX/camera_link`. Static transforms were added from
`rX/base_link` to `rX/camera_link` using a configured translation
of 0.08 m in x and 0.12 m in z, with zero rotation.

The `odom` -> `camera_link` transform lookup returned valid
transforms for all three robots. Initial frame-discovery warnings
appeared before valid results were returned.

The purpose of the prefixes is to avoid collisions between robot
frame IDs in the shared TF system.

## 5. Sensor Configuration and Rate Measurements

### 5.1 LiDAR

LiDAR messages were observed on `/r0/scan`, `/r1/scan`, and
`/r2/scan`. The LiDAR sensor is configured for a 5 Hz update rate.

Observed rates during the Gazebo/WSL sessions were lower than the
configured rate:

| Topic | Earlier observed rate | Later short measurement |
|---|---:|---:|
| `/r0/scan` | 4.0 Hz | Not individually isolated |
| `/r1/scan` | 4.0 Hz | Not individually isolated |
| `/r2/scan` | 3.4 Hz | Not individually isolated |

A later short check showed rates in the approximate range of
3.2–3.6 Hz across the three robots. These measurements are
session-specific and are not guaranteed rates.

### 5.2 RGB-D Cameras

Each robot model includes a simulated depth camera with RGB and
depth image streams.

| Parameter | Configuration |
|---|---|
| Image resolution | 320 x 240 pixels |
| Configured update rate | 10 Hz |
| Horizontal field of view | 1.047 rad |
| Near clipping distance | 0.10 m |
| Far clipping distance | 8.0 m |
| Camera frame | `rX/camera_link` |

RGB image, depth image, camera information, and depth point-cloud
topics were discovered for all three robots.

Short measurements produced the following preliminary observations:

| Sensor stream | Preliminary observed rate |
|---|---:|
| RGB, r0 | Approximately 6.7 Hz |
| RGB, r1 | Approximately 7.0 Hz |
| RGB, r2 | Stable measurement not obtained |
| Depth, across available checks | Approximately 6.5–7.2 Hz |

The measurements were collected using short-lived ROS CLI processes
in Gazebo under WSL2. Startup warnings and ROS context shutdown
errors occurred during some checks. Therefore, these values should
be treated as preliminary observations, not definitive performance
benchmarks.

The configured update rate is a target, not proof that the target
is achieved. Longer-duration measurements and performance analysis
remain necessary.

## 6. Current Scope and Limitations

The verified baseline and subsequent assignment tests cover:

- Three TurtleBot3 Burger robots spawning in one Gazebo world.
- Namespaced command, odometry, IMU, and LiDAR topics.
- Robot-specific TF frames and static robot-link transforms.
- RGB and depth image topics for all three robot models.
- Camera frame IDs and static base-to-camera transforms.
- Preliminary LiDAR and camera publication-rate measurements.
- Command isolation in the previously tested `r0` movement case.
- A custom static-world A* planner using Gazebo SDF geometry, with
  odometry-based path following and a forward LiDAR stop/resume guard.
- A simulated mission reaching ten unique waypoints, plus a separate
  obstacle-detour test.
- Software-level robot failure/reassignment testing.
- Semantic scene-graph hierarchy, plan validation, and scripted
  robot/target-unavailable event handling.

The following remain incomplete or unverified:

- A complete multi-robot mapping workflow and ground-truth map-accuracy
  evaluation.
- Nav2 configuration and navigation tests for every robot.
- Dynamic-obstacle rerouting and guaranteed collision avoidance.
- Integration of semantic events with live Gazebo motion and execution.
- Long-duration sensor-rate and simulation-performance benchmarks.

The A* planner uses static Gazebo SDF geometry rather than the saved SLAM
map. Scripted semantic events update planning state but do not demonstrate
live-motion replanning. Results must be reported within these limits.

## 7. Reproducibility Principle

Measured results, tested behavior, design intentions, and planned
features should be reported separately. A feature is not considered
verified solely because its software package is installed or because
its implementation is planned.

Rate measurements should include their observation conditions and
limitations. Future performance claims should be supported by
repeatable measurements rather than configured sensor rates alone.
