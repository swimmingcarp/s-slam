# s_slam_replay

Offline ROS 2 bag replay and frontend regression entry point for s-slam.

## Verified Datasets

### 1. Oxford Spires

**Config**

```text
s_slam_replay/config/spark_fast_lio/oxford_spires_hesai.yaml
```

**Download**

```text
Website:    https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/
Downloader: https://github.com/ori-drs/oxford_spires_dataset
```

**Replay**

```bash
./s_slam_replay/run.py \
  --input <oxford_rosbag_dir> \
  --output <output_dir> \
  --config s_slam_replay/config/spark_fast_lio/oxford_spires_hesai.yaml \
  --lidar-topic /hesai/pandar \
  --imu-topic /alphasense_driver_ros/imu \
  --lidar-frame pandar \
  --imu-frame imu_sensor_frame \
  --visualization-frame lidar \
  --read-ahead-queue-size 2000
```
