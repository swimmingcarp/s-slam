# visualization

RViz configs + launch for the s-slam system. Run it alongside the processing nodes.

## Run

```bash
source install/setup.bash
ros2 launch visualization rviz.launch.yaml                       # default: rviz/system.rviz
ros2 launch visualization rviz.launch.yaml rviz_config:=/path/to.rviz
```

`system.rviz` shows the whole pipeline (registered scan, global map, raw vs.
corrected trajectory, loop closures). It assumes **root-namespace** topics
(`/odometry`, `/cloud_registered`, `/global_map`, `/path/corrected`, …); if you run a
stage under a namespace, prefix the display topics accordingly.

## Contents

| File | For |
|------|-----|
| `rviz/system.rviz` | whole SLAM system (front end + back end) |
| `rviz/kiss_matcher_reg.rviz` | the `registration_visualizer` demo |
| `rviz/kimera_multi_initial_alignment.rviz` | the `inter_frame_alignment` tool |
