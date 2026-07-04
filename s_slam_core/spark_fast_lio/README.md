# spark_fast_lio

LiDAR-inertial odometry front end of s-slam. This ROS package is a modified
Fast-LIO2 pipeline used to estimate local odometry and publish registered point
clouds for the backend.

Operational commands and bring-up workflows are maintained in the top-level
README.

## Role

```text
LiDAR + IMU -> spark_fast_lio -> odometry + registered clouds
```

Primary inputs:

- `lidar`: `sensor_msgs/msg/PointCloud2`
- `imu`: `sensor_msgs/msg/Imu`
- TF between base, LiDAR, and IMU frames

Primary outputs:

- `odometry`: `nav_msgs/msg/Odometry`
- `path`: `nav_msgs/msg/Path`
- `cloud_registered`: registered cloud in the map/world frame
- `cloud_registered_lidar`: registered cloud in LiDAR frame
- `cloud_registered_body`: registered cloud in body frame
- `cloud_registered_base`: registered cloud in base frame, when enabled

`kiss_matcher_ros` consumes `odometry` and `cloud_registered` for loop closure.

## Config Ownership

Live sensor configs stay in this package:

```text
config/
├── avia.yaml
├── horizon.yaml
├── ouster.yaml
├── rs_fairy.yaml
└── velodyne.yaml
```

Recorded-bag presets do not belong here. They live in
`../../s_slam_replay/config/spark_fast_lio/`.

## Fairy Configuration Notes

`config/rs_fairy.yaml` and `launch/mapping_rs_fairy.launch.yaml` are for
RoboSense Fairy live use.

Important device-specific values:

- RoboSense driver must publish `PointXYZIRT` fields and IMU data.
- Fairy internal IMU/LiDAR extrinsics are read from DIFOP and written into
  `rs_fairy.yaml`. Re-read them if the LiDAR unit changes.
- The airframe mounting transform must replace the placeholder
  `base_link -> rslidar` static transform in the launch file.
- IMU covariance, blind distance, and point filtering are tuning parameters that
  need static logs and flight logs.
- Motion quality gate thresholds in `rs_fairy.yaml` must reflect the vehicle's
  real speed envelope. Keep speed and frame-step gates as catastrophic sanity
  checks, not normal UAV or vehicle speed limits.

## Add a Live Sensor Config

For a new live sensor, add a config under `config/` and keep dataset-specific
rosbag presets out of this package.

Typical fields to review:

- `preprocess.lidar_type`
- `preprocess.scan_line`
- timestamp behavior
- blind distance
- IMU noise and bias covariance
- LiDAR/IMU extrinsics
- point filtering and map voxel size

## Acknowledgement

Modified from [FAST-LIO2](https://github.com/hku-mars/FAST_LIO).
