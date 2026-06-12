# spark_fast_lio

LiDAR-inertial odometry front end of [s-slam](../README.md) — a modified Fast-LIO2.
Built as part of the s-slam workspace; see the [top-level README](../README.md).

## Run

Pre-processed ROS 2 bags: [Dropbox](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0).

```bash
source install/setup.bash

# MIT campus — Kimera-Multi dataset (10_14_acl_jackal, 10_14_hathor)
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=acl_jackal
ros2 bag play 10_14_acl_jackal       # another terminal

# Colosseum — VBR dataset (colosseo_train0)
ros2 launch spark_fast_lio mapping_vbr_colosseo.launch.yaml
ros2 bag play colosseo_train0
```

Kimera-Multi per-scene commands: [launch/README.md](launch/README.md).

## Your own dataset

Copy `config/velodyne_mit.yaml` (or `ouster_vbr.yaml`) to `config/<your>.yaml`, set
`lidar_type`, `scan_line`, `timestamp_unit`, `filter_size_map`, `extrinsic_T`,
`extrinsic_R`, remap the LiDAR/IMU topics in your launch file, then:

```bash
ros2 launch spark_fast_lio <your>.launch.yaml
```

## With loop closure (KISS-Matcher-SAM)

```bash
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
# own data: odom_topic:=<...> scan_topic:=<...>
```

## Acknowledgement

Modified from [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) (HKU MaRS Lab).
