# kiss_matcher_ros

Loop-closure / global-registration backend of [s-slam](../README.md). A flat ROS 2
(Humble) `ament_cmake` package bundling the
[KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) core library and the SLAM
nodes (pose-graph manager, loop detector, loop closure).

> Python bindings / C++ examples are not included here — install from PyPI
> (`pip install kiss-matcher`) or use upstream.

## Build

Part of the s-slam workspace — see the [top-level README](../README.md).

## Run

```bash
source install/setup.bash

# whole system (front end + this backend)
ros2 launch kiss_matcher_ros slam_in_kimera_multi.launch.yaml scene_id:=acl_jackal2 sensor_type:=velodyne16

# backend only
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml

# registration visualizer (set the PCD paths in config/params.yaml first)
ros2 launch kiss_matcher_ros visualizer.launch.yaml
```

Executables: `kiss_matcher_sam`, `run_kiss_matcher`, `registration_visualizer`, `inter_frame_alignment`.

## License

MIT — see [LICENSE](LICENSE).

## Citation

```bibtex
@inproceedings{lim2025icra-KISSMatcher,
  title={{KISS-Matcher: Fast and Robust Point Cloud Registration Revisited}},
  author={Lim, Hyungtae and Kim, Daebeom and Shin, Gunhee and Shi, Jingnan and Vizzo, Ignacio and Myung, Hyun and Park, Jaesik and Carlone, Luca},
  booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
  year={2025}
}
```
