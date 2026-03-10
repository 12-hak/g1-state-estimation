#!/usr/bin/env python3
"""
Subscribe to /cmd_vel (Twist) and send to G1 via Unitree LocoClient.
Run on the Jetson (or machine with Unitree DDS to the robot) so Nav2 cmd_vel is followed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    HAS_UNITREE = True
except ImportError:
    HAS_UNITREE = False


class CmdVelToUnitreeNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_unitree")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("network_interface", "eth0")
        self.declare_parameter("min_interval_s", 0.05)
        topic = self.get_parameter("cmd_vel_topic").value
        iface = self.get_parameter("network_interface").value
        self._min_interval = self.get_parameter("min_interval_s").value
        self._last_send = 0.0
        self._loco = None

        if not HAS_UNITREE:
            self.get_logger().error("unitree_sdk2py not found; install it on the Jetson to drive the robot from cmd_vel")
            return

        try:
            ChannelFactoryInitialize(0, iface)
            self._loco = LocoClient()
            self._loco.SetTimeout(5.0)
            self._loco.Init()
        except Exception as e:
            self.get_logger().error(f"LocoClient init failed: {e}")
            return

        self._sub = self.create_subscription(Twist, topic, self._cb, 10)
        self.get_logger().info(f"cmd_vel -> Unitree LocoClient on {topic} (iface={iface})")

    def _cb(self, msg: Twist):
        if self._loco is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_send < self._min_interval:
            return
        self._last_send = now
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vyaw = float(msg.angular.z)
        try:
            self._loco.Move(vx, vy, vyaw)
        except Exception as e:
            self.get_logger().warn(f"Move failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUnitreeNode()
    if HAS_UNITREE and node._loco is not None:
        rclpy.spin(node)
    else:
        node.get_logger().info("Exiting (no Unitree SDK or LocoClient)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
