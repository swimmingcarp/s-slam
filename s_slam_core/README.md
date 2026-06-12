# s_slam_core

Core SLAM source grouping for the s-slam workspace.

This directory is intentionally **not** a ROS package. It has no top-level
`package.xml` or `CMakeLists.txt`. `colcon` discovers the ROS packages inside it:

```text
s_slam_core/
├── spark_fast_lio/   # ROS package: spark_fast_lio
├── kiss_matcher/     # ROS package: kiss_matcher_ros
└── third_party/      # shared/vendored dependencies for the core packages
```

## Ownership

Keep algorithm code and live hardware configuration here:

- `spark_fast_lio`: LiDAR-inertial odometry front end
- `kiss_matcher_ros`: loop closure, global registration, pose graph
- `third_party`: dependencies used by the core packages

Do not put these in `s_slam_core`:

- Recorded-bag presets. Put them in `s_slam_replay`.
- Simulator worlds, bridges, CARLA/Gazebo launch files. Put them in
  `s_slam_simulation`.
- RViz config files. Put them in `s_slam_visualization`.

## Runtime Pipeline

```text
spark_fast_lio
  inputs:  LiDAR point cloud + IMU
  outputs: /odometry, /path, /cloud_registered, frame-specific registered clouds

kiss_matcher_ros
  inputs:  odometry + registered cloud
  outputs: corrected path, global map, loop-closure markers, saved results
```
