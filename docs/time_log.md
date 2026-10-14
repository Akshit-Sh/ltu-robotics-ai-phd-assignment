
### Multi-robot TF verification — 15 October 2026
- Added one robot_state_publisher per robot using the TurtleBot3 Burger URDF.
- Expanded the URDF with distinct frame prefixes: r0/, r1/, and r2/.
- Verified /tf contains odom -> base_footprint for all three robots.
- Verified /tf_static contains base_footprint -> base_link and the caster, IMU, and LiDAR link transforms for all three robots.
- Verified /tf_static has three publishers, one in each robot namespace.
