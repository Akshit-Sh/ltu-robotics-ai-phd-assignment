
### Multi-robot TF verification — 15 October 2026
- Added one robot_state_publisher per robot using the TurtleBot3 Burger URDF.
- Expanded the URDF with distinct frame prefixes: r0/, r1/, and r2/.
- Verified /tf contains odom -> base_footprint for all three robots.
- Verified /tf_static contains base_footprint -> base_link and the caster, IMU, and LiDAR link transforms for all three robots.
- Verified /tf_static has three publishers, one in each robot namespace.

### LiDAR rate baseline — 15 October 2026
- Verified that /r0/scan, /r1/scan, and /r2/scan publish LaserScan messages.
- Earlier observed rates during the Gazebo/WSL session:
  - /r0/scan: approximately 4.0 Hz.
  - /r1/scan: approximately 4.0 Hz.
  - /r2/scan: approximately 3.4 Hz.
- Later short measurements were approximately 3.2–3.6 Hz across the three robots.
- These are observed rates, not configured target rates.
- Further investigation of simulation performance and scan frequency remains pending.

### RGB-D camera integration and TF verification — 15 October 2026
- Added a simulated RGB-D depth camera to each robot model.
- Configured image resolution to 320 x 240 pixels and update rate to 10 Hz.
- Configured depth clipping range from 0.10 m to 8.0 m.
- Verified RGB image topics for all three robots:
  - /r0/camera/color/image_raw
  - /r1/camera/color/image_raw
  - /r2/camera/color/image_raw
- Verified depth image topics for all three robots:
  - /r0/camera/depth/image_raw
  - /r1/camera/depth/image_raw
  - /r2/camera/depth/image_raw
- RGB and depth image headers reported the expected frame ID, r0/camera_link, for robot r0.
- Added static transforms from each robot's base_link to its camera_link.
- Verified that odom -> camera_link transforms were returned for r0, r1, and r2.
- The camera transform checks initially showed frame-discovery warnings, followed by valid transform output.
- Preliminary observed RGB rates were approximately 6.7–7.0 Hz for r0 and r1. A complete stable RGB measurement for r2 was not obtained.
- Preliminary observed depth rates were approximately 6.5–7.2 Hz across the robots, based on the available short checks.
- These measurements were taken using short-lived ROS CLI processes in Gazebo under WSL. Startup warnings and ROS context shutdown errors appeared during some checks, so the rates should be treated as preliminary rather than definitive benchmarks.
- Camera and LiDAR performance optimization and longer-duration measurements remain pending.
