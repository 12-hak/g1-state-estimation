#!/usr/bin/env python3
"""
Relay /cmd_vel between two ROS 2 domains (e.g. 42 -> 0 for Unitree SDK on domain 0).
Run two processes: one subscribes on sub_domain and sends to UDP; one receives from UDP and publishes on pub_domain.
Usage:
  ROS_DOMAIN_ID=42 python3 cmd_vel_domain_relay.py --mode sub --udp-port 9871 &
  ROS_DOMAIN_ID=0  python3 cmd_vel_domain_relay.py --mode pub --udp-port 9871 &
Or use the launch file cmd_vel_domain_relay.launch.py which spawns both with the correct domains.
"""

import argparse
import struct
import socket
import sys


def pack_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    return struct.pack("@dddddd", linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)


def unpack_twist(data):
    if len(data) != 48:
        return None
    return struct.unpack("@dddddd", data)


def run_sub(port: int, topic: str):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node("cmd_vel_relay_sub")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def cb(msg: Twist):
        data = pack_twist(
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )
        sock.sendto(data, ("127.0.0.1", port))

    sub = node.create_subscription(Twist, topic, cb, 10)
    node.get_logger().info(f"Relay sub: {topic} -> UDP localhost:{port} (use ROS_DOMAIN_ID=42)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    sock.close()


def run_pub(port: int, topic: str):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node("cmd_vel_relay_pub")
    pub = node.create_publisher(Twist, topic, 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("127.0.0.1", port))
    except OSError as e:
        node.get_logger().error(f"Bind UDP :{port} failed: {e}")
        sys.exit(1)

    node.get_logger().info(f"Relay pub: UDP localhost:{port} -> {topic} (use ROS_DOMAIN_ID=0)")
    twist = Twist()
    while rclpy.ok():
        try:
            sock.settimeout(0.5)
            data, _ = sock.recvfrom(64)
        except socket.timeout:
            continue
        except Exception:
            break
        t = unpack_twist(data)
        if t is None:
            continue
        twist.linear.x, twist.linear.y, twist.linear.z = t[0], t[1], t[2]
        twist.angular.x, twist.angular.y, twist.angular.z = t[3], t[4], t[5]
        pub.publish(twist)
    sock.close()
    node.destroy_node()
    rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description="Relay cmd_vel between ROS 2 domains via UDP")
    p.add_argument("--mode", choices=["sub", "pub"], required=True, help="sub=subscribe on this domain and send UDP; pub=receive UDP and publish on this domain")
    p.add_argument("--udp-port", type=int, default=9871, help="UDP port for relay (default 9871)")
    p.add_argument("--topic", default="/cmd_vel", help="ROS topic name (default /cmd_vel)")
    args = p.parse_args()
    if args.mode == "sub":
        run_sub(args.udp_port, args.topic)
    else:
        run_pub(args.udp_port, args.topic)


if __name__ == "__main__":
    main()
