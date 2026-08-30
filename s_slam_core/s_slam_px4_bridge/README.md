# PX4 DDS Odometry Bridge

This package sends the FAST-LIO high-rate predicted state to PX4 as
`px4_msgs/msg/VehicleOdometry` on `/fmu/in/vehicle_visual_odometry`. It is a
ROS 2/uXRCE-DDS bridge, not a MAVLink bridge: the hand-written UDP handshake
and `TIMESYNC` logic in `debug_logs/phase3_pipeline.py` are intentionally not
used.

The bridge uses `px4_odometry`, which atomically carries
the base-frame pose, world-frame linear velocity, their covariance, and reset
counter. The pose is initialized from each LiDAR-corrected FAST-LIO state and
then propagated at IMU rate. The bridge selects the source sample nearest each
30 Hz target time and never repeats an older sample to fill a sensor gap.

PX4 receives position, attitude, linear velocity, position/attitude/velocity
variance, source timestamp, and reset counter in FRD frames. `angular_velocity`
is `NaN` because FAST-LIO does not estimate it as an independent state, and
`quality` is zero because PX4 marks it unused.

## Prerequisites

For ROS 2 Humble, Micro XRCE-DDS Agent `v2.4.2` is the documented compatible
agent version. That is an **agent** version, not the PX4 firmware version. The
actual PX4 firmware release is still required: clone the matching `px4_msgs`
branch into this workspace and source its installation before running the
bridge. The bridge intentionally fails its CMake configuration without that
dependency, rather than succeeding without an executable.

```bash
source /opt/ros/humble/setup.bash
cd /home/jaden/workspace/s-slam
colcon build --packages-up-to s_slam_px4_bridge --symlink-install
colcon test --packages-select s_slam_px4_bridge --event-handlers console_direct+
```

## Flight Configuration

The supplied configuration enables the bridge. Calibrate both row-major
rotations in
`config/px4_odometry_bridge.yaml`:

- `px4_world_from_source_world` maps the FAST-LIO map frame into PX4's local
  FRD frame.
- `source_body_from_px4_body` maps PX4's body FRD axes into the FAST-LIO odom
  child-frame axes.

The two origins must also be established during the same stationary startup.
Do not treat the identity matrices as a generic ENU/FLU-to-PX4 conversion.
`source_world_frame` and `source_child_frame` are checked for every input;
the child frame must be the calibrated PX4 vehicle-reference origin. Do not
silently substitute a LiDAR or IMU frame with an unmodelled lever arm.

FAST-LIO increments the embedded reset counter both for estimator recovery and
when gravity alignment changes its public map frame. This prevents PX4 from
interpreting either event as a physical measurement jump.

The on-companion `px4_odometry` handoff is reliable with a bounded history of
ten states. The PX4 output remains best-effort to match the uXRCE-DDS input
path; there is no application-level retransmission of stale odometry.

After the PX4 uXRCE-DDS client is connected to the companion over Ethernet,
start the bridge:

```bash
ros2 launch s_slam_px4_bridge px4_odometry_bridge.launch.yaml
```

PX4 owns uXRCE time synchronization. The bridge sets the publication
`timestamp` from the companion ROS clock and forwards the FAST-LIO header time
as `timestamp_sample`; therefore the sensor driver and companion must share a
valid time base before external-vision fusion is enabled.
