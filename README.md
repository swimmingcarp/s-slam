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
| Core front end | `spark_fast_lio` | Live LiDAR/IMU processing, live sensor configs, Fairy bring-up launch | Rosbag dataset presets, RViz configs |
| Core back end | `kiss_matcher_ros` | Loop closure, pose graph, corrected map/path, result save callback | Bag playback, simulator setup |
| Replay | `s_slam_replay` | Recorded-bag presets, whole-system replay launch, replay result export | Simulator worlds, live hardware configs |
| Simulation | `s_slam_simulation` | Future CARLA/Gazebo/Ignition launch, worlds, models, bridges | Recorded-bag replay presets, RViz config files |
| Visualization | `s_slam_visualization` | RViz configs and RViz launch | Algorithm config, simulator assets |

## Step-by-Step Setup

This guide targets a clean Ubuntu 22.04 / Jammy machine. Run the commands in
order. The first build downloads pinned third-party source code, including GTSAM
if no system copy is available and small_gicp for the backend, so network access
to GitHub is required once.

### 1. Install System Tools

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release \
  build-essential cmake git python3-pip locales
sudo add-apt-repository -y universe
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

### 2. Install ROS 2 Humble

Follow the same apt repository setup used by the official ROS 2 Humble Ubuntu
Debian-package guide:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

Check ROS:

```bash
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null
colcon --help >/dev/null
```

### 3. Prepare the Workspace

Use system Python and system compilers. This avoids Anaconda/Conda paths leaking
into ROS and Boost.

```bash
cd /home/jaden/workspace/s-slam

export PATH=$HOME/.local/bin:/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
export CC=/usr/bin/cc
export CXX=/usr/bin/c++
source /opt/ros/humble/setup.bash
```

### 4. Install Dependencies

Install native libraries used directly by CMake and by the vendored GTSAM
fallback:

```bash
sudo apt install -y libboost-all-dev libeigen3-dev libpcl-dev libflann-dev \
  liblz4-dev libtbb-dev libomp-dev
```

Initialize `rosdep` once per machine. If it was already initialized, the first
command may print an error; continuing is OK.

```bash
sudo rosdep init || true
rosdep update
```

Install dependencies declared by the package manifests:

```bash
rosdep install --from-paths . --ignore-src -r -y
```

### 5. Check CMake

Ubuntu 22.04 ships CMake 3.22, which is sufficient for this workspace:

```bash
cmake --version
```

The printed version must be `3.22` or newer.

### 6. Build Everything

For a clean first build:

```bash
cd /home/jaden/workspace/s-slam
source /opt/ros/humble/setup.bash
scripts/build.sh --clean
scripts/build.sh
source install/setup.bash
```

`scripts/build.sh --clean` removes `build/`, `install/`, and `log/`, then exits.
Run `scripts/build.sh` after it to compile.

For normal rebuilds after source or config changes:

```bash
scripts/build.sh
source install/setup.bash
```

`scripts/build.sh` builds packages sequentially and uses at most half of the
detected CPU cores per package, capped at 4 jobs. On WSL or low-memory machines,
use the safest mode:

```bash
BUILD_JOBS=1 scripts/build.sh
```

## Replay Workflow

Run one end-to-end local regression from an existing ROS 2 bag:

```bash
./s_slam_replay/run.py --input <rosbag_dir> --output <output_dir>
```

By default, replay uses the Fairy front-end config:
`s_slam_core/spark_fast_lio/config/rs_fairy.yaml`.

For another LiDAR, pass the matching `spark_fast_lio` config:

```bash
./s_slam_replay/run.py --input <rosbag_dir> --config <config_yaml> --output <output_dir>
```

The command starts the front end, replays the bag, records `/odometry`, and
writes `report.md`, `metrics.json`, logs, and the output odometry bag under the
requested output directory.

If LiDAR and IMU were recorded into separate sibling bag directories, pass their
parent directory as `<rosbag_dir>`.

The selected FAST-LIO YAML remains the single source of algorithm and sensor
tuning. If the YAML defines the same node parameter as a frame override, ROS
parameter-file order may still make the YAML the final source of truth.

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

## RoboSense Fairy Bring-Up

Run the RoboSense driver separately, then start the Fairy LIO front end:

```bash
ros2 launch spark_fast_lio mapping_rs_fairy.launch.yaml
```

Expected RoboSense driver topics:

```text
/rslidar_points     sensor_msgs/msg/PointCloud2 with PointXYZIRT fields
/rslidar_imu_data   sensor_msgs/msg/Imu
```

Current Fairy launch status:

- It is a bench/bring-up launch, not a finished flight configuration.
- `launch/mapping_rs_fairy.launch.yaml` currently uses an identity
  `base_link -> rslidar` static transform.
- `config/rs_fairy.yaml` uses the Fairy DIFOP IMU/LiDAR extrinsics read from
  the test unit on the Jetson.

Before flight or real device validation:

- Update the real RoboSense driver config: point type `XYZIRT`, IMU parsing
  enabled, Fairy lidar type, LiDAR clock settings, and IMU port.
- Re-read Fairy DIFOP `IMU_CALIB_DATA` and update `extrinsic_T` and
  `extrinsic_R` if the LiDAR unit changes.
- Measure and replace the `base_link -> rslidar` mounting transform in
  `mapping_rs_fairy.launch.yaml`.
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
