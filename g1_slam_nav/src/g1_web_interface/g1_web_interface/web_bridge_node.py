"""
WebSocket bridge node: streams ROS2 topics to web clients and serves the frontend.
Handles bidirectional communication for map visualization, navigation commands,
and waypoint management.
"""

import json
import asyncio
import threading
import math
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from g1_msgs.msg import SlamStatus, NavigationCommand, WaypointList
from g1_msgs.srv import SaveMap, LoadMap

try:
    import websockets
    from websockets.asyncio.server import serve as ws_serve
except ImportError:
    import websockets
    ws_serve = websockets.serve


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')

        self.declare_parameter('ws_port', 9090)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('frontend_path', '')
        self.declare_parameter('map_downsample', 4)
        self.declare_parameter('map_publish_rate', 1.0)
        self.declare_parameter('pose_publish_rate', 10.0)

        self.ws_port = self.get_parameter('ws_port').value
        self.http_port = self.get_parameter('http_port').value
        self.frontend_path = self.get_parameter('frontend_path').value
        self.map_downsample = self.get_parameter('map_downsample').value

        # State cache
        self._pose = None
        self._map_grid = None
        self._slam_status = None
        self._planned_path = None
        self._clients = set()
        self._lock = threading.Lock()
        self._loop = None

        # QoS for map (transient local)
        map_qos = QoSProfile(depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.create_subscription(Odometry, 'slam/odom', self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, map_qos)
        self.create_subscription(SlamStatus, 'slam/status', self._status_cb, 10)
        self.create_subscription(Path, 'nav/planned_path', self._path_cb, 10)

        # Publishers
        self._goal_pub = self.create_publisher(PoseStamped, 'nav/goal', 10)
        self._cmd_pub = self.create_publisher(NavigationCommand, 'nav/command', 10)
        self._wp_pub = self.create_publisher(WaypointList, 'nav/waypoints', 10)

        # Service clients
        self._save_map_client = self.create_client(SaveMap, 'save_map')
        self._load_map_client = self.create_client(LoadMap, 'load_map')

        # Timers for rate-limited broadcasting
        pose_rate = self.get_parameter('pose_publish_rate').value
        map_rate = self.get_parameter('map_publish_rate').value
        self.create_timer(1.0 / pose_rate, self._broadcast_pose)
        self.create_timer(1.0 / map_rate, self._broadcast_map)

        self.get_logger().info(
            f'Web bridge: WS={self.ws_port}, HTTP={self.http_port}')

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
                'yaw': yaw,
            }

    def _map_cb(self, msg: OccupancyGrid):
        info = msg.info
        ds = self.map_downsample
        w = info.width // ds
        h = info.height // ds
        data = []
        for gy in range(h):
            for gx in range(w):
                orig_x = gx * ds
                orig_y = gy * ds
                val = msg.data[orig_y * info.width + orig_x]
                data.append(int(val))

        with self._lock:
            self._map_grid = {
                'width': w,
                'height': h,
                'resolution': info.resolution * ds,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'data': data,
            }

    def _status_cb(self, msg: SlamStatus):
        with self._lock:
            self._slam_status = {
                'mapping_active': msg.mapping_active,
                'localization_valid': msg.localization_valid,
                'map_points': msg.map_point_count,
                'loop_closures': msg.loop_closure_count,
                'quality': msg.odometry_quality,
            }

    def _path_cb(self, msg: Path):
        points = []
        for pose in msg.poses:
            points.append([pose.pose.position.x, pose.pose.position.y])
        with self._lock:
            self._planned_path = points

    def _broadcast_pose(self):
        if self._loop is None:
            return
        with self._lock:
            if self._pose is None:
                return
            payload = {'type': 'pose', **self._pose}
            if self._slam_status:
                payload['status'] = self._slam_status
            data = json.dumps(payload)
        asyncio.run_coroutine_threadsafe(self._send_all(data), self._loop)

    def _broadcast_map(self):
        if self._loop is None:
            return
        with self._lock:
            if self._map_grid is None:
                return
            payload = {'type': 'map', **self._map_grid}
            if self._planned_path:
                payload['path'] = self._planned_path
            data = json.dumps(payload)
        asyncio.run_coroutine_threadsafe(self._send_all(data), self._loop)

    async def _send_all(self, data: str):
        dead = set()
        for ws in list(self._clients):
            try:
                await ws.send(data)
            except Exception:
                dead.add(ws)
        self._clients -= dead

    async def _handle_ws(self, websocket):
        self._clients.add(websocket)
        self.get_logger().info(f'Client connected ({len(self._clients)} total)')
        try:
            async for message in websocket:
                await self._process_command(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.discard(websocket)
            self.get_logger().info(f'Client disconnected ({len(self._clients)} total)')

    async def _process_command(self, raw: str):
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return

        cmd_type = msg.get('type', '')

        if cmd_type == 'navigate':
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(msg['x'])
            goal.pose.position.y = float(msg['y'])
            yaw = float(msg.get('yaw', 0.0))
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self._goal_pub.publish(goal)
            self.get_logger().info(f'Goal set: ({msg["x"]:.2f}, {msg["y"]:.2f})')

        elif cmd_type == 'cancel':
            cmd = NavigationCommand()
            cmd.command_type = NavigationCommand.CANCEL
            self._cmd_pub.publish(cmd)

        elif cmd_type == 'waypoints':
            wp_list = WaypointList()
            wp_list.header.stamp = self.get_clock().now().to_msg()
            wp_list.loop = msg.get('loop', False)
            for wp in msg.get('points', []):
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x = float(wp['x'])
                ps.pose.position.y = float(wp['y'])
                yaw = float(wp.get('yaw', 0.0))
                ps.pose.orientation.z = math.sin(yaw / 2.0)
                ps.pose.orientation.w = math.cos(yaw / 2.0)
                wp_list.waypoints.append(ps)
            self._wp_pub.publish(wp_list)

        elif cmd_type == 'save_map':
            if self._save_map_client.service_is_ready():
                req = SaveMap.Request()
                req.filename = msg.get('name', '')
                future = self._save_map_client.call_async(req)
                future.add_done_callback(self._handle_save_result)

        elif cmd_type == 'load_map':
            if self._load_map_client.service_is_ready():
                req = LoadMap.Request()
                req.filepath = msg.get('path', '')
                future = self._load_map_client.call_async(req)
                future.add_done_callback(self._handle_load_result)

    def _handle_save_result(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Map save: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')

    def _handle_load_result(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Map load: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Map load failed: {e}')

    async def _run_ws_server(self):
        async with ws_serve(self._handle_ws, '0.0.0.0', self.ws_port):
            self.get_logger().info(f'WebSocket server running on port {self.ws_port}')
            await asyncio.Future()  # run forever

    def _run_http_server(self):
        if self.frontend_path and os.path.isdir(self.frontend_path):
            handler = partial(SimpleHTTPRequestHandler, directory=self.frontend_path)
        else:
            handler = SimpleHTTPRequestHandler

        server = HTTPServer(('0.0.0.0', self.http_port), handler)
        self.get_logger().info(f'HTTP server running on port {self.http_port}')
        server.serve_forever()

    def start_servers(self):
        # HTTP server in background thread
        http_thread = threading.Thread(target=self._run_http_server, daemon=True)
        http_thread.start()

        # WebSocket server in asyncio event loop (background thread)
        ws_thread = threading.Thread(target=self._run_ws_loop, daemon=True)
        ws_thread.start()

    def _run_ws_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        loop.run_until_complete(self._run_ws_server())


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    node.start_servers()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
