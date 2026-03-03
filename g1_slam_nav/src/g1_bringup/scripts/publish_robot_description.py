#!/usr/bin/env python3
"""Publish URDF to /robot_description (transient_local) for Foxglove 3D panel. Republishes periodically for late-joining clients."""

import sys
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


def main():
    if len(sys.argv) < 2:
        print("Usage: publish_robot_description.py <path_to_urdf>", file=sys.stderr)
        sys.exit(1)
    urdf_path = sys.argv[1]
    with open(urdf_path, "r") as f:
        urdf = f.read()

    rclpy.init()
    node = rclpy.create_node("robot_description_publisher")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(String, "/robot_description", qos)
    msg = String()
    msg.data = urdf

    def publish_once():
        pub.publish(msg)
        node.get_logger().info("Published /robot_description", throttle_duration_sec=5.0)

    node.create_timer(2.0, publish_once)
    publish_once()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
