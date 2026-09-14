# PX4 DDS Odometry Bridge

This package sends the FAST-LIO high-rate predicted state to PX4 as
`px4_msgs/msg/VehicleOdometry` on `/fmu/in/vehicle_visual_odometry`. It is a
ROS 2/uXRCE-DDS bridge, not a MAVLink bridge: the hand-written UDP handshake
and `TIMESYNC` logic in `debug_logs/phase3_pipeline.py` are intentionally not
used.

The bridge uses `px4_odometry`, which atomically carries the base-frame pose,
world-frame linear velocity, body rate, their covariance, and reset counter.
The pose is initialized from each LiDAR-corrected FAST-LIO state and then
propagated at IMU rate.

The bridge publishes on its own 30 Hz timer. Each slot carries the newest
source state advanced to the moment it is sent, using that state's own linear
velocity and body rate, with `timestamp_sample` moved by the same interval. PX4
therefore receives a pose that is valid when it arrives instead of one that is
as old as the handoff happened to be.

Propagation is capped at 30 ms, which is an error budget for holding velocity
and body rate constant: about 4 mm at the 8 m/s² the front end declares as its
acceleration limit, and about a degree of attitude at the angular acceleration
a multirotor reaches. Whenever the front end keeps up its newest state is a few
milliseconds old and the cap never applies. An older state is still sent, but
stamped at the last time its propagation is trustworthy, so PX4 fuses it as the
delayed measurement it is. A slot whose newest state has already been sent is
left empty instead: re-sending it would have PX4 fuse one measurement as
several and would hide a stalled source from its own external-vision timeout.

The reported variances are the source state's own. The message carries no
position/velocity cross-covariance, so the growth over the propagation interval
cannot be computed from it; over the capped interval the omitted term is at
most the velocity variance times 0.9 ms².

PX4 receives position, attitude, linear velocity, body rate,
position/attitude/velocity variance, sample timestamp, and reset counter in FRD
frames. `quality` is zero because PX4 marks it unused.

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
ten states, of which the bridge keeps only the newest. There is no output
queue, so the delay PX4 sees is the age of the newest state rather than a
backlog that never drains. A complete source or executor stall appears as an
output gap. The PX4 output remains best-effort to match the uXRCE-DDS input
path; there is no application-level retransmission of stale odometry.

After the PX4 uXRCE-DDS client is connected to the companion over Ethernet,
start the bridge:

```bash
ros2 launch s_slam_px4_bridge px4_odometry_bridge.launch.yaml
```

PX4 owns uXRCE time synchronization. The bridge sets the publication
`timestamp` from the companion ROS clock and derives `timestamp_sample` from
the FAST-LIO header time advanced by the interval the state was held. That
interval is measured on the companion clock alone, so pacing and propagation
are correct whatever offset the sensor clock carries. `timestamp_sample` itself
only means anything to PX4 once the sensor driver and the companion share a
time base: a RoboSense driver running `use_lidar_clock` publishes the LiDAR's
own free-running counter, which is not the companion epoch. Establish that time
base before enabling external-vision fusion.
