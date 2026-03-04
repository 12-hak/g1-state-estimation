#!/usr/bin/env python3
"""Publish TF from map frame to base_link using Point-LIO Odometry (aft_mapped_to_init)."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PointLioOdomToTf(Node):
    def __init__(self):
        super().__init__("point_lio_odom_to_tf")
        self.declare_parameter("odom_topic", "aft_mapped_to_init")
        self.declare_parameter("map_frame", "camera_init")
        self.declare_parameter("base_frame", "base_link")
        odom_topic = self.get_parameter("odom_topic").value
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10
        )
        self.get_logger().info(
            f"Publishing TF {self._map_frame} -> {self._base_frame} from {odom_topic}"
        )

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = PointLioOdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
