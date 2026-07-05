# Replay Launch Files

This directory contains launch files for recorded-bag replay. Operational command
lines are maintained in the top-level README.

## Files

| File | Scope |
|------|-------|
| `spark_fast_lio_replay.launch.yaml` | generic front-end replay launch used by `s_slam_replay/run.py` |
| `slam_in_kimera_multi.launch.yaml` | full Kimera-Multi replay pipeline: `spark_fast_lio` front end plus `kiss_matcher_ros` backend |
| `save_result.launch.yaml` | helper that triggers backend result export through `save_dir` |
| `mapping_kimera_multi.launch.yaml` | Kimera-Multi front end only |
| `mapping_mit_campus.launch.yaml` | MIT campus front end only |
| `mapping_vbr_colosseo.launch.yaml` | VBR Colosseo front end only |
| `mapping_dcist_rrg.launch.yaml` | DCIST RRG front end only |

## Naming

Use `replay_id` as the public recorded-bag identifier. Deprecated aliases
`scene_id` and `dataset_id` may still exist for backward compatibility, but new
commands should be documented only in the top-level README.

## Preset Notes

Some Kimera-Multi recordings need frame overrides such as `lidar_frame` or
`imu_frame`. Keep the canonical command lines and expected bag names in the
top-level README so replay instructions stay in one place.
