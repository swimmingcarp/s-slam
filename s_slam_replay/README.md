# s_slam_replay

ROS bag replay package for s-slam.

This package stores recorded-bag presets and result-export helpers. The actual
replay workflow is maintained in the top-level README.

This is **not** the simulation package. Simulator worlds, CARLA/Gazebo bridges,
robot models, and simulator GUI launch files belong in `../s_slam_simulation`.

## Role

```text
recorded rosbag
  -> s_slam_replay launch preset
  -> spark_fast_lio + kiss_matcher_ros
  -> live ROS topics + saved trajectory/map artifacts
```

`s_slam_replay` owns:

- recorded-bag launch presets
- dataset-specific front-end configs
- dataset-specific backend configs
- result-save helper launch

It does not own:

- live hardware configs
- simulator assets
- RViz config files

## Directory Layout

```text
s_slam_replay/
├── config/
│   ├── spark_fast_lio/
│   │   ├── kimera_multi/      # Kimera-Multi bag presets
│   │   ├── dcist_rrg/         # DCIST RRG bag presets
│   │   ├── velodyne_mit.yaml
│   │   └── ouster_vbr.yaml
│   └── kiss_matcher/
│       └── kimera_multi/      # backend presets by sensor type
├── launch/
│   ├── slam_in_kimera_multi.launch.yaml
│   ├── mapping_kimera_multi.launch.yaml
│   ├── mapping_mit_campus.launch.yaml
│   ├── mapping_vbr_colosseo.launch.yaml
│   ├── mapping_dcist_rrg.launch.yaml
│   └── save_result.launch.yaml
└── scripts/
    └── publish_save_dir.py
```

## Public Arguments

| Argument | Used by | Meaning |
|----------|---------|---------|
| `replay_id` | front end, backend, save helper | namespace, preset id, default result directory name |
| `sensor_type` | backend config selection | KISS-Matcher preset, for example `velodyne16` or `ouster64` |
| `robot_name` | front-end frame wiring | original robot name inside the bag |
| `save_map_pcd` | backend | save accumulated map when `save_dir` is triggered |
| `save_in_kitti_format` | backend | save keyframe scans plus TUM/KITTI trajectories |
| `result_seq_name` | backend | result subdirectory name |

Deprecated aliases `scene_id` and `dataset_id` may still exist in launch files
for compatibility. New documentation and scripts should use `replay_id`.

## Outputs

Replay result files are written by the KISS-Matcher backend after the save helper
triggers `save_dir`:

```text
<output_dir>/<result_seq_name>/
├── poses_tum.txt
├── poses_kitti.txt
├── scans/
│   └── 000000.pcd ...
└── <result_seq_name>_map.pcd
```

| File | Format | Purpose |
|------|--------|---------|
| `poses_tum.txt` | `timestamp x y z qx qy qz qw` | trajectory evaluation |
| `poses_kitti.txt` | one `3x4` pose matrix per line | KITTI-compatible trajectory tools |
| `scans/*.pcd` | PCD | keyframe clouds |
| `<result_seq_name>_map.pcd` | PCD | accumulated corrected map |

The `.pcd` and `.txt` files are saved artifacts. RViz displays live ROS topics
while the replay is running; it does not load these files directly.

## Add a New Replay Preset

1. Add or copy a front-end config under `config/spark_fast_lio/<dataset>/`.
2. Add a backend config under `config/kiss_matcher/<dataset>/` if loop-closure
   tuning differs by sensor or dataset.
3. Add or update a launch file under `launch/` to set topic remaps, frames,
   namespace, and config paths.
4. Use `replay_id` as the public argument. Keep old aliases only for backward
   compatibility.
5. Document the bag name, required topics, expected result path, and command in
   the top-level README.
