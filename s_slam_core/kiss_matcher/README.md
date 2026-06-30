# kiss_matcher_ros

Loop-closure and pose-graph backend of s-slam. This ROS package bundles the
KISS-Matcher registration code with a ROS 2 SLAM node.

Operational commands, backend-only runs, replay usage, and result-save commands
are maintained in the top-level README.

## Role

```text
front-end odometry + registered cloud
  -> kiss_matcher_sam
  -> corrected trajectory + loop closures + global map
```

Primary inputs are relative topic names. Launch files remap them to the
front-end outputs, usually `/odometry` and `/cloud_registered`.

- `odom`: `nav_msgs/msg/Odometry`
- `cloud`: `sensor_msgs/msg/PointCloud2`

Primary outputs:

- `path/original`: front-end trajectory
- `path/corrected`: pose-graph corrected trajectory
- `global_map`: accumulated corrected map
- `curr_scan`: current scan used by the backend
- `loop_detection`: accepted loop-closure marker
- `loop_detection_radius`: loop-candidate search radius marker
- `pose_stamped`: current corrected pose
- `lc/*`: loop-closure debug clouds

If the node is launched in a namespace, these outputs are relative to that
namespace, for example `/acl_jackal2/path/corrected`.

## Result Export Interface

The backend saves results when it receives a `std_msgs/msg/String` on the
relative topic `save_dir`. The message data is the output root directory.

Exported files:

```text
<output_dir>/<result_seq_name>/
├── poses_tum.txt       # timestamp x y z qx qy qz qw
├── poses_kitti.txt     # one 3x4 pose matrix per line
├── scans/*.pcd         # keyframe point clouds
└── <result_seq_name>_map.pcd
```

Result-export switches are controlled by launch arguments instead of static
config files, so replay can enable exports while backend-only runs stay off by
default.

## Configuration

`config/slam_config.yaml` owns backend behavior such as:

- keyframe selection
- loop candidate search
- local registration
- global registration
- map visualization update frequency

Dataset or sensor-specific backend presets belong in `../../s_slam_replay` when
they are only used for recorded-bag replay.

## Executables

- `kiss_matcher_sam`: ROS 2 pose-graph backend
- `run_kiss_matcher`: standalone registration runner
- `registration_visualizer`: PCD registration visualizer
- `inter_frame_alignment`: pairwise alignment tool

## License

MIT. See [LICENSE](LICENSE).

## Citation

```bibtex
@inproceedings{lim2025icra-KISSMatcher,
  title={{KISS-Matcher: Fast and Robust Point Cloud Registration Revisited}},
  author={Lim, Hyungtae and Kim, Daebeom and Shin, Gunhee and Shi, Jingnan and Vizzo, Ignacio and Myung, Hyun and Park, Jaesik and Carlone, Luca},
  booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
  year={2025}
}
```
