# s_slam_simulation

Simulation package for s-slam.

This package is currently a scaffold. It reserves stable locations for future
simulator integrations such as CARLA, Gazebo, or Ignition. Operational simulation
workflows should be documented in the top-level README once a runnable simulator
entry exists.

## Role

`s_slam_simulation` owns simulator-side assets and launch files:

- simulator server/client launch
- simulator bridge launch, for example `carla_ros_bridge`
- simulated vehicle/drone models
- simulated LiDAR/IMU/camera definitions
- simulator worlds, maps, scenarios, and routes
- simulator-specific topic remaps and `use_sim_time` settings

It does not own:

- recorded-bag replay presets; use `../s_slam_replay`
- RViz config files; use `../s_slam_visualization`
- live hardware configs; use `../s_slam_core/spark_fast_lio`

## Visualization Boundary

Simulation can include simulator visualization. For example, a CARLA integration
may open the Unreal viewport, and a Gazebo integration may open the Gazebo GUI.
That belongs here because it is part of the simulator.

RViz visualization of SLAM topics belongs in `s_slam_visualization`.

```text
CARLA/Gazebo/Ignition viewport -> s_slam_simulation
RViz map/path/cloud displays   -> s_slam_visualization
Rosbag replay validation       -> s_slam_replay
Algorithm implementation       -> s_slam_core
```

## Directory Layout

```text
s_slam_simulation/
├── launch/   # simulator and bridge launch files
├── config/   # simulator bridge, sensor, route, scenario config
├── urdf/     # robot/drone descriptions used by simulation
├── worlds/   # Gazebo/Ignition worlds or world-like assets
└── models/   # simulator model assets
```

## Expected Future CARLA Layout

One reasonable CARLA integration would look like this:

```text
s_slam_simulation/
├── launch/
│   ├── carla_server.launch.yaml
│   ├── carla_bridge.launch.yaml
│   └── carla_slam.launch.yaml
├── config/
│   └── carla/
│       ├── airy_sensor_rig.yaml
│       ├── topic_remap.yaml
│       └── town10_loop.yaml
└── models/
```

If a future simulation launch optionally starts RViz, it should include RViz
configs from `s_slam_visualization` instead of storing RViz files here.

## Current Status

There is no runnable simulator launch in this package yet. The package exists so
future simulation work has a clear home and does not get mixed into replay or
visualization.
