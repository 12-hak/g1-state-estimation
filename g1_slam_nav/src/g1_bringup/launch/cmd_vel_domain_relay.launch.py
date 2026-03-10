"""
Run cmd_vel relay between domain 42 (our stack) and domain 0 (Unitree SDK).
Sub process: ROS_DOMAIN_ID=42, subscribes to /cmd_vel, forwards to UDP.
Pub process: ROS_DOMAIN_ID=0, receives from UDP, publishes to /cmd_vel.

Run on the Jetson (where Unitree SDK listens on domain 0). Our stack (client/Jetson) publishes on domain 42;
this relay republishes on domain 0 so the Unitree driver receives it.

  ros2 launch g1_bringup cmd_vel_domain_relay.launch.py
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    prefix = get_package_prefix("g1_bringup")
    script = os.path.join(prefix, "lib", "g1_bringup", "cmd_vel_domain_relay.py")
    if not os.path.isfile(script):
        script = os.path.join(prefix, "libexec", "g1_bringup", "cmd_vel_domain_relay.py")
    python = sys.executable if sys.executable else "python3"

    env_sub = {**os.environ, "ROS_DOMAIN_ID": "42"}
    env_pub = {**os.environ, "ROS_DOMAIN_ID": "0"}

    sub_proc = ExecuteProcess(
        cmd=[python, script, "--mode", "sub", "--udp-port", "9871"],
        env=env_sub,
        output="screen",
        name="cmd_vel_relay_sub",
    )
    pub_proc = ExecuteProcess(
        cmd=[python, script, "--mode", "pub", "--udp-port", "9871"],
        env=env_pub,
        output="screen",
        name="cmd_vel_relay_pub",
    )
    return LaunchDescription([sub_proc, pub_proc])
