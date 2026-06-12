# s-slam

LiDAR-inertial SLAM workspace for UAV/robot bring-up, rosbag replay, and SLAM
result visualization.

This README is the single operational guide for the workspace. Environment setup,
build commands, replay commands, result export, RViz, and bring-up workflows are
kept here so the same process is not duplicated across package READMEs.

## Runtime Pipeline

```text
LiDAR + IMU
  -> spark_fast_lio          # LIO front end: odometry + registered scans
  -> kiss_matcher_ros        # loop closure + pose-graph backend
  -> corrected trajectory + global map
```

## Repository Layout

```text
s-slam/
├── s_slam_core/             # source grouping; not a ROS package
│   ├── spark_fast_lio/      # ROS package: spark_fast_lio
│   ├── kiss_matcher/        # ROS package: kiss_matcher_ros
│   └── third_party/         # shared/vendored core dependencies
├── s_slam_replay/           # ROS package: rosbag replay presets + result export
├── s_slam_simulation/       # ROS package: simulator assets/launch; currently a scaffold
├── s_slam_visualization/    # ROS package: RViz configs and RViz launch
└── LICENSE                  # mixed licenses; see License
```

`s_slam_core` is only a directory boundary. `colcon` discovers the ROS packages
inside it recursively.

## Package Roles

| Area | ROS package | Owns | Does not own |
|------|-------------|------|--------------|
| Core front end | `spark_fast_lio` | Live LiDAR/IMU processing, live sensor configs, Airy bring-up launch | Rosbag dataset presets, RViz configs |
| Core back end | `kiss_matcher_ros` | Loop closure, pose graph, corrected map/path, result save callback | Bag playback, simulator setup |
| Replay | `s_slam_replay` | Recorded-bag presets, whole-system replay launch, replay result export | Simulator worlds, live hardware configs |
| Simulation | `s_slam_simulation` | Future CARLA/Gazebo/Ignition launch, worlds, models, bridges | Recorded-bag replay presets, RViz config files |
| Visualization | `s_slam_visualization` | RViz configs and RViz launch | Algorithm config, simulator assets |

## Environment

Prerequisites:

- ROS 2 Humble and `colcon`
- PCL, GTSAM, Eigen3, FLANN, TBB, OpenMP, LZ4
- CMake `>= 3.24` and `< 4.0`

Install missing ROS/system dependencies from package manifests:

```bash
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

Install CMake if the system version is too old:

```bash
pip install --user "cmake>=3.24,<4"
```

Use system Python and system compilers when building. This matters on machines
where Anaconda appears before `/usr/bin` in `PATH`; ROS Humble expects Python
3.10 and system Boost libraries.

```bash
cd /home/jaden/workspace/sanshan-workspace/s-slam

export PATH=$HOME/.local/bin:/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
export CC=/usr/bin/cc
export CXX=/usr/bin/c++
source /opt/ros/humble/setup.bash
```

Use the same environment in every terminal before running commands:

```bash
cd /home/jaden/workspace/sanshan-workspace/s-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Build

Clean build:

```bash
colcon build --cmake-clean-cache --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_C_COMPILER=/usr/bin/cc \
  -DCMAKE_CXX_COMPILER=/usr/bin/c++
```

Incremental rebuild after source or launch/config changes:

```bash
colcon build --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_C_COMPILER=/usr/bin/cc \
  -DCMAKE_CXX_COMPILER=/usr/bin/c++
```

After every build:

```bash
source install/setup.bash
```

Expected packages:

```bash
colcon list --names-only
# kiss_matcher_ros
# s_slam_replay
# s_slam_simulation
# s_slam_visualization
# spark_fast_lio
```

If CMake previously cached Anaconda paths, rebuild once with
`--cmake-clean-cache`.

## Replay Workflow

Use replay for algorithm verification and regression testing. Replay is not
simulation; it feeds recorded rosbag data through the live SLAM pipeline.

Pre-processed ROS 2 bags are currently documented here:
[Dropbox](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0).

Terminal 1, start the full SLAM pipeline:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch s_slam_replay slam_in_kimera_multi.launch.yaml \
  replay_id:=acl_jackal2 \
  sensor_type:=velodyne16
```

Terminal 2, play the bag:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag play 12_08_acl_jackal2
```

Terminal 3, after playback finishes, save backend results:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch s_slam_replay save_result.launch.yaml \
  replay_id:=acl_jackal2 \
  output_dir:=/tmp/s_slam_replay_results
```

`save_result.launch.yaml` publishes a `std_msgs/msg/String` to
`/<replay_id>/save_dir`. KISS-Matcher receives that message and writes results
under the requested directory.

Replay result files:

```text
/tmp/s_slam_replay_results/acl_jackal2/
├── poses_tum.txt
├── poses_kitti.txt
├── scans/
│   └── 000000.pcd ...
└── acl_jackal2_map.pcd
```

| File | Format | Purpose |
|------|--------|---------|
| `poses_tum.txt` | `timestamp x y z qx qy qz qw` | trajectory evaluation with `evo` |
| `poses_kitti.txt` | one `3x4` pose matrix per line | KITTI-compatible trajectory tools |
| `scans/*.pcd` | PCD | keyframe clouds |
| `<replay_id>_map.pcd` | PCD | accumulated corrected map |

Disable large result export for exploratory runs:

```bash
ros2 launch s_slam_replay slam_in_kimera_multi.launch.yaml \
  replay_id:=acl_jackal2 \
  sensor_type:=velodyne16 \
  save_map_pcd:=false \
  save_in_kitti_format:=false
```

## Replay Evaluation

With ground truth:

```bash
evo_ape tum ground_truth.tum /tmp/s_slam_replay_results/acl_jackal2/poses_tum.txt -a --plot
evo_rpe tum ground_truth.tum /tmp/s_slam_replay_results/acl_jackal2/poses_tum.txt -a --plot
```

Without ground truth:

- compare `poses_tum.txt` across commits
- inspect drift and loop-closure behavior in RViz
- compare generated map PCDs visually or with point-cloud metrics

## RViz Workflow

Start RViz in a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch s_slam_visualization rviz.launch.yaml
```

Use a custom RViz config:

```bash
ros2 launch s_slam_visualization rviz.launch.yaml rviz_config:=/path/to/file.rviz
```

Root-namespace topics:

```text
/odometry
/cloud_registered
/path/original
/path/corrected
/global_map
/loop_detection
```

Namespaced replay topics, for example `replay_id:=acl_jackal2`:

```text
/acl_jackal2/odometry
/acl_jackal2/cloud_registered
/acl_jackal2/path/original
/acl_jackal2/path/corrected
/acl_jackal2/global_map
/acl_jackal2/loop_detection
```

Saved `.pcd` and `.txt` replay artifacts are evaluation outputs. RViz does not
load them directly unless a node republishes them as ROS topics.

## Front-End-Only Replay Presets

Use these to debug `spark_fast_lio` before enabling the backend:

```bash
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=acl_jackal
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=acl_jackal2
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=sparkal1
```

Kimera-Multi presets that need frame overrides:

```bash
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=apis     lidar_frame:=ouster_link imu_frame:=camera_imu_optical_frame
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=hathor   lidar_frame:=velodyne
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=sobek    lidar_frame:=ouster_link
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=sparkal2 lidar_frame:=velodyne
ros2 launch s_slam_replay mapping_kimera_multi.launch.yaml replay_id:=thoth    lidar_frame:=ouster_link imu_frame:=camera_imu_optical_frame
```

Other front-end presets:

```bash
ros2 launch s_slam_replay mapping_mit_campus.launch.yaml replay_id:=acl_jackal
ros2 launch s_slam_replay mapping_vbr_colosseo.launch.yaml
ros2 launch s_slam_replay mapping_dcist_rrg.launch.yaml
```

## Backend-Only Workflow

Use this when the front end or another odometry source is already publishing
odometry and registered scans:

```bash
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml \
  odom_topic:=/odometry \
  scan_topic:=/cloud_registered
```

Enable backend result export:

```bash
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml \
  save_map_pcd:=true \
  save_in_kitti_format:=true \
  result_seq_name:=sequence
```

Trigger backend-only save:

```bash
ros2 topic pub --once /save_dir std_msgs/msg/String "{data: '/tmp/s_slam_results'}"
```

For a namespaced backend:

```bash
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml \
  namespace:=acl_jackal2 \
  save_map_pcd:=true \
  save_in_kitti_format:=true \
  result_seq_name:=acl_jackal2

ros2 topic pub --once /acl_jackal2/save_dir std_msgs/msg/String "{data: '/tmp/s_slam_results'}"
```

## RoboSense Airy Bring-Up

Run the RoboSense driver separately, then start the Airy LIO front end:

```bash
ros2 launch spark_fast_lio mapping_rs_airy.launch.yaml
```

Expected RoboSense driver topics:

```text
/rslidar_points     sensor_msgs/msg/PointCloud2 with PointXYZIRT fields
/rslidar_imu_data   sensor_msgs/msg/Imu
```

Current Airy launch status:

- It is a bench/bring-up launch, not a finished flight configuration.
- `launch/mapping_rs_airy.launch.yaml` currently uses an identity
  `base_link -> rslidar` static transform.
- `config/rs_airy.yaml` currently uses placeholder Airy LiDAR/IMU extrinsics.

Before flight or real device validation:

- Update the real RoboSense driver config: point type `XYZIRT`, IMU parsing
  enabled, Airy lidar type, LiDAR clock settings, and IMU port.
- Read Airy DIFOP `IMU_CALIB_DATA`, convert it to the LiDAR-w.r.t-IMU transform,
  and replace `extrinsic_T` and `extrinsic_R` in `rs_airy.yaml`.
- Measure and replace the `base_link -> rslidar` mounting transform in
  `mapping_rs_airy.launch.yaml`.
- Tune IMU noise, blind distance, and point filtering from static logs and
  flight logs.
- Validate `/odometry`, `/path`, and `/cloud_registered` in RViz before flight.

## Simulation Status

`s_slam_simulation` is currently a scaffold. There is no runnable CARLA, Gazebo,
or Ignition launch in this workspace yet.

Simulation ownership rule:

```text
CARLA/Gazebo/Ignition viewport -> s_slam_simulation
RViz map/path/cloud displays   -> s_slam_visualization
Rosbag replay validation       -> s_slam_replay
Algorithm implementation       -> s_slam_core
```

## Documentation Map

Package READMEs describe ownership, inputs/outputs, and file layout only. They
do not duplicate environment setup or operational workflows.

- [s_slam_core](s_slam_core/README.md): core source grouping and ownership rules
- [spark_fast_lio](s_slam_core/spark_fast_lio/README.md): LIO front end
- [kiss_matcher_ros](s_slam_core/kiss_matcher/README.md): loop-closure backend
- [s_slam_replay](s_slam_replay/README.md): replay package structure and outputs
- [s_slam_simulation](s_slam_simulation/README.md): simulator scaffold
- [s_slam_visualization](s_slam_visualization/README.md): RViz package structure

## License

This workspace combines code under different licenses. `s_slam_core/kiss_matcher`
is MIT. `s_slam_core/spark_fast_lio` is GPL-2.0. Redistribution of combined work
must comply with GPL-2.0. See [LICENSE](LICENSE).
