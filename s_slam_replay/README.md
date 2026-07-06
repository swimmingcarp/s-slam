# s_slam_replay

ROS bag replay package for s-slam.

This package stores recorded-bag presets and result-export helpers. The actual
replay workflow is maintained in the top-level README.

This is **not** the simulation package. Simulator worlds, CARLA/Gazebo bridges,
robot models, and simulator GUI launch files belong in `../s_slam_simulation`.

## Role

```text
recorded rosbag
  -> s_slam_replay generic launch
  -> spark_fast_lio + kiss_matcher_ros
  -> live ROS topics + saved trajectory/map artifacts
```

`s_slam_replay` owns:

- recorded-bag launch presets
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
│   ├── spark_fast_lio_replay.launch.yaml
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

Run one end-to-end local regression from an existing ROS 2 bag:

```bash
./s_slam_replay/run.py --input <rosbag_dir> --output <output_dir>
```

By default, replay uses the Fairy front-end config:
`s_slam_core/spark_fast_lio/config/rs_fairy.yaml`.

For another LiDAR, pass a direct `spark_fast_lio` config:

```bash
./s_slam_replay/run.py --input <rosbag_dir> --config <config_yaml> --output <output_dir>
```

If LiDAR and IMU were recorded into separate sibling bag directories, pass their
parent directory as `<rosbag_dir>`.

The wrapper starts `spark_fast_lio`, replays the bag, records `/odometry`, and
writes:

```text
<output_dir>/
├── odom/              # recorded replay output
├── logs/              # frontend, recorder, and bag-play logs
├── input_info.txt
├── metrics.json
├── report.md
└── run_manifest.json
```

`report.md` and `metrics.json` also include a frontend resource summary sampled
during bag playback: CPU (cores and percent), resident memory, and thread count,
each reported as average and peak, plus system CPU load for context.

## Add a New Replay Preset

1. Add or copy a front-end config under `config/spark_fast_lio/<dataset>/` only
   when the dataset genuinely needs separate FAST-LIO tuning.
2. Add a backend config under `config/kiss_matcher/<dataset>/` if loop-closure
   tuning differs by sensor or dataset.
3. Add or update a launch file under `launch/` only when the generic replay
   launch is not enough to set topic remaps, frames, namespace, and config paths.
4. Use `replay_id` as the public argument. Keep old aliases only for backward
   compatibility.
