#!/usr/bin/python3

import time

import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("publish_save_dir")

    replay_id = node.declare_parameter("replay_id", "acl_jackal2").value
    output_dir = node.declare_parameter("output_dir", "/tmp/s_slam_replay_results").value
    wait_timeout_sec = node.declare_parameter("wait_timeout_sec", 5.0).value

    topic = f"/{replay_id}/save_dir" if replay_id else "/save_dir"
    publisher = node.create_publisher(String, topic, 10)

    start = time.monotonic()
    while rclpy.ok() and publisher.get_subscription_count() == 0:
        if time.monotonic() - start >= wait_timeout_sec:
            node.get_logger().warn(f"No subscribers discovered on {topic}; publishing anyway")
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    message = String()
    message.data = output_dir

    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.2)

    node.get_logger().info(f"Requested result save to {output_dir} via {topic}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
