# s_slam_core

Core SLAM source grouping for the s-slam workspace.

This directory is intentionally **not** a ROS package. It has no top-level
`package.xml` or `CMakeLists.txt`. `colcon` discovers the ROS packages inside it:

```text
s_slam_core/
├── spark_fast_lio/   # ROS package: spark_fast_lio
├── kiss_matcher/     # ROS package: kiss_matcher_ros
├── s_slam_interfaces/ # ROS package: shared message definitions
├── s_slam_px4_bridge/ # ROS package: FAST-LIO to PX4 DDS odometry bridge
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
  outputs: odometry, path, cloud_registered, frame-specific registered clouds

kiss_matcher_ros
  inputs:  odom + cloud; launch files remap these relative names to front-end topics
  outputs: corrected path, global map, loop-closure markers, saved results
```

## Live Fairy Bringup Contract

Use the system launch when running the RoboSense Fairy front end and
KISS-Matcher backend together:

```bash
ros2 launch kiss_matcher_ros slam_rs_fairy.launch.yaml
```

Default topic contract:

```text
SDK -> front end:
  /rslidar_points
  /rslidar_imu_data

front end -> backend:
  /odometry          -> backend relative topic odom
  /cloud_registered  -> backend relative topic cloud
```

Default TF ownership:

```text
map -> odom         backend, dynamic
odom -> base_link   front end, dynamic
base_link -> lidar  static mounting transform
```

Do not publish a static transform between `odom` and `base_link`; that relation
belongs to the front end.
