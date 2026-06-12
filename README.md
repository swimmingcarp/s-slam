# s-slam

LiDAR-inertial SLAM = **`spark_fast_lio`** (LIO odometry front end, a modified Fast-LIO2)
→ **`kiss_matcher_ros`** (loop closure + global registration via
[KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) + GTSAM pose-graph optimization).

```
LiDAR + IMU → spark_fast_lio → (odometry, registered cloud) → kiss_matcher_sam → globally-consistent map
```

## Layout

```
s-slam/
├── kiss_matcher/     # → kiss_matcher_ros  (loop closure + registration backend)
├── spark_fast_lio/   # → spark_fast_lio    (LIO front end)
├── visualization/    # → visualization     (all RViz configs + rviz launch)
├── third_party/      # shared deps — see third_party/README.md
└── LICENSE           # mixed: kiss_matcher MIT, spark_fast_lio GPL-2.0
```

## Prerequisites

- ROS 2 Humble + colcon, and: PCL, GTSAM, Eigen3, FLANN, TBB, OpenMP, LZ4
  (`rosdep install --from-paths . --ignore-src -r -y`).
- CMake **≥ 3.24 and < 4.0** (system 3.22 is too old; 4.x breaks the fetched deps):
  ```bash
  pip install --user "cmake>=3.24,<4"
  ```
- CMake must use the **system** `python3` (Anaconda's lacks `catkin_pkg`):
  `export PATH=/usr/bin:$PATH`, or pass `-DPython3_EXECUTABLE=/usr/bin/python3`.

## Build & run

```bash
export PATH=$HOME/.local/bin:/usr/bin:$PATH
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
# whole system (front end + back end):
ros2 launch kiss_matcher_ros slam_in_kimera_multi.launch.yaml scene_id:=acl_jackal2 sensor_type:=velodyne16
```

Run RViz on its own (the processing launches don't start it):

```bash
ros2 launch visualization rviz.launch.yaml
```

Per-package usage: [kiss_matcher](kiss_matcher/README.md) · [spark_fast_lio](spark_fast_lio/README.md) · [visualization](visualization/README.md).

## License

Mixed: `kiss_matcher/` is MIT, `spark_fast_lio/` is GPL-2.0. Redistribution of the
combined work must comply with the GPL. See [LICENSE](LICENSE).
