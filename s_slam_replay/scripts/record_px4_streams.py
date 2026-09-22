#!/usr/bin/python3
"""Record both ends of the PX4 odometry handoff during a replay.

`ros2 bag record` cannot capture the 200 Hz source: `px4_odometry` is published
with a ten-sample history so the bridge only ever reads the newest state, and a
recorder that falls behind silently loses and reorders samples. This node keeps
a deep reliable queue and appends plain numbers, so the recorded cadence is the
frontend's own. The bridge output is captured on the same QoS PX4 subscribes
with, so an empty publication slot is recorded exactly as PX4 would see it.
"""

import csv
import math
import os
import time

import rclpy
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from s_slam_interfaces.msg import Px4Odometry

SOURCE_QUEUE_DEPTH = 20000
OUTPUT_QUEUE_DEPTH = 2000


class Px4StreamRecorder(Node):
    def __init__(self):
        super().__init__("record_px4_streams")
        output_dir = self.declare_parameter("output_dir", "/tmp/s_slam_replay_px4").value
        source_topic = self.declare_parameter("source_topic", "px4_odometry").value
        output_topic = self.declare_parameter(
            "output_topic", "/fmu/in/vehicle_visual_odometry"
        ).value

        os.makedirs(output_dir, exist_ok=True)
        self.source_file = open(f"{output_dir}/px4_source.csv", "w", newline="")
        self.source_writer = csv.writer(self.source_file)
        self.source_writer.writerow(
            [
                "stamp_s",
                "arrival_s",
                "position_x",
                "position_y",
                "position_z",
                "quaternion_x",
                "quaternion_y",
                "quaternion_z",
                "quaternion_w",
                "velocity_x",
                "velocity_y",
                "velocity_z",
                "position_variance_x",
                "position_variance_y",
                "position_variance_z",
                "velocity_variance_x",
                "velocity_variance_y",
                "velocity_variance_z",
                "reset_counter",
                "finite",
            ]
        )
        self.output_file = open(f"{output_dir}/px4_bridge_output.csv", "w", newline="")
        self.output_writer = csv.writer(self.output_file)
        self.output_writer.writerow(
            [
                "timestamp_us",
                "timestamp_sample_us",
                "arrival_s",
                "pose_frame",
                "velocity_frame",
                "position_x",
                "position_y",
                "position_z",
                "quaternion_w",
                "quaternion_x",
                "quaternion_y",
                "quaternion_z",
                "velocity_x",
                "velocity_y",
                "velocity_z",
                "angular_velocity_x",
                "angular_velocity_y",
                "angular_velocity_z",
                "position_variance_x",
                "position_variance_y",
                "position_variance_z",
                "orientation_variance_x",
                "orientation_variance_y",
                "orientation_variance_z",
                "velocity_variance_x",
                "velocity_variance_y",
                "velocity_variance_z",
                "reset_counter",
                "quality",
                "finite",
            ]
        )

        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=SOURCE_QUEUE_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Best-effort like PX4, so a slot the bridge deliberately leaves empty is
        # recorded as an absence rather than filled in by a retransmission. The
        # depth is this recorder's own backlog, not part of that contract: at
        # depth one a publication is overwritten whenever the 200 Hz source
        # callback is mid-write, and the recorded gap is the instrument.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=OUTPUT_QUEUE_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Px4Odometry, source_topic, self.handleSource, source_qos)
        self.create_subscription(
            VehicleOdometry, output_topic, self.handleBridgeOutput, output_qos
        )
        self.source_count = 0
        self.output_count = 0
        self.get_logger().info(
            f"Recording {source_topic} and {output_topic} into {output_dir}"
        )

    def handleSource(self, message):
        position = message.pose.position
        orientation = message.pose.orientation
        velocity = message.linear_velocity
        values = [
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            velocity.x,
            velocity.y,
            velocity.z,
        ]
        variances = [message.pose_covariance[index * 6 + index] for index in range(3)]
        variances += [
            message.linear_velocity_covariance[index * 3 + index] for index in range(3)
        ]
        self.source_writer.writerow(
            [
                f"{message.header.stamp.sec + message.header.stamp.nanosec * 1e-9:.6f}",
                f"{time.time():.6f}",
                *values,
                *variances,
                message.reset_counter,
                int(all(math.isfinite(value) for value in values + variances)),
            ]
        )
        self.source_count += 1

    def handleBridgeOutput(self, message):
        values = [
            *message.position,
            *message.q,
            *message.velocity,
            *message.angular_velocity,
        ]
        variances = [
            *message.position_variance,
            *message.orientation_variance,
            *message.velocity_variance,
        ]
        self.output_writer.writerow(
            [
                message.timestamp,
                message.timestamp_sample,
                f"{time.time():.6f}",
                message.pose_frame,
                message.velocity_frame,
                *values,
                *variances,
                message.reset_counter,
                message.quality,
                int(all(math.isfinite(value) for value in values + variances)),
            ]
        )
        self.output_count += 1

    def close(self):
        self.get_logger().info(
            f"Recorded {self.source_count} source and {self.output_count} bridge samples"
        )
        self.source_file.close()
        self.output_file.close()


def main():
    rclpy.init()
    node = Px4StreamRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()


if __name__ == "__main__":
    main()
