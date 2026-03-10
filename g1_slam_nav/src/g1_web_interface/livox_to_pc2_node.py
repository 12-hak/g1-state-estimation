"""
Convert livox_ros_driver2 CustomMsg (/livox/lidar) to PointCloud2 (pointcloud)
for SLAM-IT's slam_node (KISS-ICP). Applies the same flip and filters as
livox_bridge_node in g1_sensor_bridge.
"""
from __future__ import annotations

import struct
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

try:
    from livox_ros_driver2.msg import CustomMsg
    _HAS_LIVOX = True
except ImportError:
    try:
        from livox_interfaces.msg import CustomMsg
        _HAS_LIVOX = True
    except ImportError:
        _HAS_LIVOX = False


_LIDAR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

_PC2_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class LivoxToPC2Node(Node):
    def __init__(self):
        super().__init__('livox_to_pc2_node')

        self.declare_parameter('flip_lidar', True)
        self.declare_parameter('body_filter_radius', 0.3)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('lidar_offset_x', 0.10)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', 0.60)
        self.declare_parameter('frame_id', 'lidar_link')

        self.flip = self.get_parameter('flip_lidar').value
        self.body_r2 = self.get_parameter('body_filter_radius').value ** 2
        self.min_r = self.get_parameter('min_range').value
        self.max_r = self.get_parameter('max_range').value
        self.lidar_offset = (
            self.get_parameter('lidar_offset_x').value,
            self.get_parameter('lidar_offset_y').value,
            self.get_parameter('lidar_offset_z').value,
        )
        self.frame_id = self.get_parameter('frame_id').value
        self._last_pc_stamp_ns = 0

        self.pc_pub = self.create_publisher(PointCloud2, 'pointcloud', _PC2_PUB_QOS)
        self.imu_relay = self.create_publisher(Imu, 'imu/body', 50)

        if _HAS_LIVOX:
            self.lidar_sub = self.create_subscription(
                CustomMsg, '/livox/lidar', self._lidar_cb, _LIDAR_QOS)
        else:
            self.get_logger().error(
                'livox_ros_driver2 message types not found. '
                'Build livox_ros_driver2 in this workspace or source it.')

        self.imu_sub = self.create_subscription(
            Imu, '/livox/imu', self._imu_cb, 50)

        self._publish_static_tf()
        self.get_logger().info(
            f'livox_to_pc2_node ready (flip={self.flip}, '
            f'range={self.min_r}-{self.max_r}m)')

    def _publish_static_tf(self):
        tf_bc = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = self.frame_id
        ox, oy, oz = self.lidar_offset
        tf.transform.translation.x = ox
        tf.transform.translation.y = oy
        tf.transform.translation.z = oz
        if self.flip:
            tf.transform.rotation.w = 0.0
            tf.transform.rotation.x = 1.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = 0.0
        else:
            tf.transform.rotation.w = 1.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = 0.0
        tf_bc.sendTransform(tf)

    def _imu_cb(self, msg: Imu):
        msg.header.frame_id = 'imu_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_relay.publish(msg)

    def _lidar_cb(self, msg: CustomMsg):
        pts_xyz = []
        for pt in msg.points:
            x, y, z = pt.x, pt.y, pt.z
            if self.flip:
                y = -y
                z = -z
            r2 = x * x + y * y + z * z
            dist = math.sqrt(r2)
            body_r2 = x * x + y * y
            if dist < self.min_r or dist > self.max_r:
                continue
            if body_r2 < self.body_r2:
                continue
            pts_xyz.append((x, y, z))

        if not pts_xyz:
            return

        pc2 = PointCloud2()
        pc2.header.frame_id = self.frame_id
        driver_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._last_pc_stamp_ns = max(self._last_pc_stamp_ns, driver_ns)
        pc2.header.stamp.sec = self._last_pc_stamp_ns // 10**9
        pc2.header.stamp.nanosec = int(self._last_pc_stamp_ns % 10**9)
        pc2.height = 1
        pc2.width = len(pts_xyz)
        pc2.is_dense = True
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * pc2.width
        pc2.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        buf = bytearray(pc2.row_step)
        for i, (x, y, z) in enumerate(pts_xyz):
            offset = i * 12
            struct.pack_into('<fff', buf, offset, x, y, z)
        pc2.data = bytes(buf)
        self.pc_pub.publish(pc2)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxToPC2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
