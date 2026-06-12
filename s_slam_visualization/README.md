# s_slam_visualization

RViz visualization package for s-slam.

This package owns RViz config files and the RViz launch entry point. Operational
commands are maintained in the top-level README.

## Role

`s_slam_visualization` displays ROS topics. It does not run SLAM, play bags,
start simulators, or evaluate trajectories.

Typical displays:

- odometry
- raw/front-end path
- corrected/backend path
- registered scan
- global map
- loop-closure markers
- TF tree

## RViz Configs

```text
rviz/
├── system.rviz
├── kiss_matcher_reg.rviz
└── kimera_multi_initial_alignment.rviz
```

| File | Use |
|------|-----|
| `system.rviz` | whole SLAM system: odometry, path, registered cloud, global map, loop markers |
| `kiss_matcher_reg.rviz` | KISS-Matcher registration visualizer |
| `kimera_multi_initial_alignment.rviz` | inter-frame alignment tool |

## Topic Namespaces

`system.rviz` is a generic config. Root-namespace SLAM topics use names such as:

```text
/odometry
/cloud_registered
/path/original
/path/corrected
/global_map
/loop_detection
```

Replay launches usually run under `replay_id`, for example `acl_jackal2`. In that
case, displays should point to namespaced topics such as:

```text
/acl_jackal2/odometry
/acl_jackal2/cloud_registered
/acl_jackal2/path/original
/acl_jackal2/path/corrected
/acl_jackal2/global_map
/acl_jackal2/loop_detection
```

## Relationship to Replay Outputs

Replay result files are saved as:

```text
poses_tum.txt
poses_kitti.txt
scans/*.pcd
*_map.pcd
```

Those files are evaluation artifacts. RViz does not read them directly unless a
node publishes them as ROS topics.

If offline result viewing becomes a required workflow, add a small publisher that
converts:

```text
*_map.pcd      -> sensor_msgs/msg/PointCloud2
poses_tum.txt  -> nav_msgs/msg/Path
```

The RViz config for that viewer should still live in this package.
