# Kimera-Multi per-scene launch commands

```bash
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=acl_jackal
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=acl_jackal2
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=sparkal1
```

Scenes needing extra frames:

```bash
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=apis     lidar_frame:=ouster_link imu_frame:=camera_imu_optical_frame
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=hathor   lidar_frame:=velodyne
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=sobek    lidar_frame:=ouster_link
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=sparkal2 lidar_frame:=velodyne
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:=thoth    lidar_frame:=ouster_link imu_frame:=camera_imu_optical_frame
```
