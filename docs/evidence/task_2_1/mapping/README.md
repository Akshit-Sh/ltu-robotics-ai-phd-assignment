# Task 2.1 — Mapping Evidence

## Implementation
- Library: slam_toolbox
- Mode: mapping
- Input scan topic: /r0/scan
- Odometry frame: r0/odom
- Base frame: r0/base_footprint
- Map frame: map
- Configured resolution: 0.05 m/cell

## Saved map
- Occupancy image: inspection_map.pgm
- Metadata: inspection_map.yaml
- Visualisation: inspection_map.png
- Dimensions: 152 x 160 cells
- Nominal physical extent: 7.60 x 8.00 m
- Approximate image-pixel classification: occupied 3.4%, free 60.1%, other/unknown 36.5%

The pixel percentages are approximate image-based classifications, not independently validated occupancy-grid statistics.

## Scope and limitations
The saved map and configuration demonstrate a robot-specific 2D mapping pipeline using r0's LiDAR and odometry. Multi-robot map merging was not implemented or demonstrated. Map accuracy against ground truth was not independently evaluated.
