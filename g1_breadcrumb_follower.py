import socket
import struct
import time
import sys
import threading
import json
import http.server
import socketserver
import numpy as np
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_, unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class VisualizerHandler(http.server.SimpleHTTPRequestHandler):
    app_instance = None
    
    def do_GET(self):
        if self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            with self.app_instance.lock:
                data = {
                    "pos": self.app_instance.current_pos.tolist(),
                    "yaw": float(self.app_instance.current_yaw),
                    "path": [p.tolist() for p in self.app_instance.recorded_path],
                    "recording": self.app_instance.is_recording,
                    "playing": self.app_instance.is_playing,
                    "udp_ok": self.app_instance.received_udp
                }
            self.wfile.write(json.dumps(data).encode())
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(self.app_instance.HTML_PAGE.encode())
        else:
            self.send_error(404)

class BreadcrumbFollower:
    HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>G1 Path Visualizer</title>
    <style>
        body { margin: 0; background: #0f172a; color: #f8fafc; font-family: 'Inter', sans-serif; overflow: hidden; }
        #info { position: absolute; top: 20px; left: 20px; background: rgba(15, 23, 42, 0.8); padding: 20px; border-radius: 12px; border: 1px solid #334155; z-index: 10; backdrop-filter: blur(8px); }
        .stat { margin-bottom: 8px; font-size: 14px; }
        .val { font-weight: bold; color: #38bdf8; }
        .tag { display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 12px; font-weight: bold; margin-right: 5px; }
        .tag-on { background: #15803d; color: #bbf7d0; }
        .tag-off { background: #334155; color: #94a3b8; }
        #canvas { width: 100vw; height: 100vh; cursor: move; }
        .legend { margin-top: 15px; font-size: 12px; display: flex; align-items: center; gap: 10px; }
        .dot { width: 10px; height: 10px; border-radius: 50%; }
        h1 { margin: 0 0 10px 0; font-size: 18px; color: #fff; }
    </style>
</head>
<body>
    <div id="info">
        <h1>G1 Path Follower</h1>
        <div class="stat">Status: <span id="status" class="val">Idle</span></div>
        <div class="stat">Position: <span id="pos" class="val">0.0, 0.0</span></div>
        <div class="stat">Points: <span id="pts" class="val">0</span></div>
        <div id="tags"></div>
        <div class="legend"><div class="dot" style="background:#38bdf8;box-shadow:0 0 5px #38bdf8"></div> Recorded Path</div>
        <div class="legend"><div class="dot" style="background:#4ade80;box-shadow:0 0 5px #4ade80"></div> Robot Position</div>
    </div>
    <canvas id="canvas"></canvas>

    <script>
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        let data = { pos: [0,0], yaw: 0, path: [], recording:false, playing:false, udp_ok:false };
        let offset = { x: 0, y: 0 };
        let scale = 50; // pixels per meter
        let autoZoom = true;

        function resize() {
            canvas.width = window.innerWidth;
            canvas.height = window.innerHeight;
            offset.x = canvas.width / 2;
            offset.y = canvas.height / 2;
        }
        window.onresize = resize;
        resize();

        async function fetchData() {
            try {
                const res = await fetch('/data');
                data = await res.json();
                updateUI();
            } catch (e) {}
            setTimeout(fetchData, 100);
        }

        function updateUI() {
            document.getElementById('pos').innerText = `${data.pos[0].toFixed(2)}, ${data.pos[1].toFixed(2)}`;
            document.getElementById('pts').innerText = data.path.length;
            document.getElementById('status').innerText = data.playing ? "PLAYING" : (data.recording ? "RECORDING" : "IDLE");
            
            let tags = "";
            tags += `<span class="tag ${data.recording ? 'tag-on' : 'tag-off'}">REC</span>`;
            tags += `<span class="tag ${data.playing ? 'tag-on' : 'tag-off'}">PLAY</span>`;
            tags += `<span class="tag ${data.udp_ok ? 'tag-on' : 'tag-off'}">LOCALIZER</span>`;
            document.getElementById('tags').innerHTML = tags;
        }

        function draw() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Grid
            ctx.strokeStyle = '#1e293b';
            ctx.lineWidth = 1;
            for(let x = (offset.x % scale); x < canvas.width; x += scale) {
                ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
            }
            for(let y = (offset.y % scale); y < canvas.height; y += scale) {
                ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
            }

            // Path
            if (data.path.length > 1) {
                ctx.strokeStyle = '#38bdf8';
                ctx.lineWidth = 3;
                ctx.shadowBlur = 10;
                ctx.shadowColor = '#38bdf8';
                ctx.beginPath();
                data.path.forEach((p, i) => {
                    const x = offset.x + p[0] * scale;
                    const y = offset.y - p[1] * scale;
                    if (i === 0) ctx.moveTo(x, y);
                    else ctx.lineTo(x, y);
                });
                ctx.stroke();
                ctx.shadowBlur = 0;
            }

            // Robot
            const rx = offset.x + data.pos[0] * scale;
            const ry = offset.y - data.pos[1] * scale;
            
            ctx.save();
            ctx.translate(rx, ry);
            ctx.rotate(-data.yaw); // Invert for canvas coords
            
            // Glow
            ctx.fillStyle = '#4ade80';
            ctx.shadowBlur = 15;
            ctx.shadowColor = '#4ade80';
            ctx.beginPath();
            ctx.arc(0, 0, 8, 0, Math.PI * 2);
            ctx.fill();
            
            // Direction Triangle
            ctx.beginPath();
            ctx.moveTo(12, 0);
            ctx.lineTo(-6, -6);
            ctx.lineTo(-6, 6);
            ctx.closePath();
            ctx.fill();
            ctx.restore();

            requestAnimationFrame(draw);
        }

        fetchData();
        draw();

        // Drag to pan
        let isPanning = false;
        canvas.onmousedown = () => isPanning = true;
        window.onmouseup = () => isPanning = false;
        window.onmousemove = (e) => {
            if (isPanning) {
                offset.x += e.movementX;
                offset.y += e.movementY;
            }
        };
        canvas.onwheel = (e) => {
            const zoom = e.deltaY > 0 ? 0.9 : 1.1;
            scale *= zoom;
        };
    </script>
</body>
</html>
    """

    def __init__(self, iface):
        self.iface = iface
        self.recorded_path = []
        self.is_recording = False
        self.is_playing = False
        
        self.current_pos = np.zeros(2)
        self.current_yaw = 0.0
        self.front_obstacle_dist = 10.0
        
        # Remote buttons
        self.l1_pressed = False
        self.l2_pressed = False
        self.up_pressed = False
        self.down_pressed = False
        
        # Data reception flags for diagnostics
        self.received_dds = False
        self.received_udp = False
        
        # State locks
        self.lock = threading.Lock()
        
        # SDK components
        ChannelFactoryInitialize(0, self.iface)
        
        # Remote subscribe (G1 uses rt/lf/lowstate for wireless_remote)
        self.lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_sub.Init(self._low_state_callback, 10)
        
        # Fallback: Sport mode state subscribe (For default position)
        self.sport_state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sport_state_sub.Init(self._sport_state_callback, 10)
        
        # Loco client for commands
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(5.0)
        self.loco_client.Init()
        
        # UDP listener for G1Localizer (G1S4 format)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("0.0.0.0", 9870))
        self.udp_sock.settimeout(0.5)
        
        # Threads
        threading.Thread(target=self._udp_listener_loop, daemon=True).start()
        threading.Thread(target=self._diagnostic_loop, daemon=True).start()
        
        # Web Visualizer Server
        VisualizerHandler.app_instance = self
        self.server = socketserver.TCPServer(("0.0.0.0", 8080), VisualizerHandler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

        print(f"Breadcrumb Follower V2 Initialized on {self.iface}")
        print("Visualizer available at http://localhost:8080")
        print("Listening for Remote on rt/lf/lowstate")
        print("Listening for Position on UDP:9870 (G1Localizer)")
        print("L1 + Up: Start/Stop Recording | L2 + Down: Playback")

    def _diagnostic_loop(self):
        while True:
            time.sleep(5)
            with self.lock:
                status = f"[STATUS] DDS: {'OK' if self.received_dds else 'MISSING'}, UDP: {'OK' if self.received_udp else 'MISSING'}"
                if self.is_recording:
                    status += f" | RECORDING: {len(self.recorded_path)} pts"
                print(status)

    def _udp_listener_loop(self):
        # magical G1S4 packet: magic(4), timestamp(8), pos_raw(12), pos_icp(12), velocity(8), ...
        # Total header is ~308 bytes, but we only need magic + bits
        while True:
            try:
                data, _ = self.udp_sock.recvfrom(2048)
                if len(data) >= 42 and data[0:4] == b"G1S4":
                    self.received_udp = True
                    # position_icp is at offset 4 + 8 + 12 = 24
                    # yaw is at offset 4 + 8 + 12 + 12 + 8 + 116 + 116 = 276 (approximately)
                    # Actually, easier to use imu_quat at offset 276
                    
                    # Unpack: magic(4s), ts(d), pos_raw(3f), pos_icp(3f)
                    pos_icp = struct.unpack('<fff', data[24:36])
                    
                    # imu_quat at offset 308 - 4 - 32 - 12 = 260
                    # Let's check struct size: 4 + 8 + 12 + 12 + 8 + 116 + 116 + 16 + 12 + 4 = 308
                    # imu_quat (4*4=16 bytes) is at 4+8+12+12+8+116+116 = 276
                    quat = struct.unpack('<ffff', data[276:292]) # w, x, y, z
                    
                    with self.lock:
                        self.current_pos = np.array([pos_icp[0], pos_icp[1]])
                        self.current_yaw = self._get_yaw(quat)
                        
                        if self.is_recording:
                            self._add_breadcrumb()
                            
            except socket.timeout:
                continue
            except Exception as e:
                print(f"UDP Error: {e}")

    def _low_state_callback(self, msg: LowState_):
        self.received_dds = True
        d1 = msg.wireless_remote[2]
        d2 = msg.wireless_remote[3]
        
        l1 = (d1 >> 1) & 1
        l2 = (d1 >> 5) & 1
        up = (d2 >> 4) & 1
        down = (d2 >> 6) & 1
        
        with self.lock:
            if l1 and up and not (self.l1_pressed and self.up_pressed):
                self._toggle_recording()
            
            if l2 and down and not (self.l2_pressed and self.down_pressed):
                self._start_playback()
            
            self.l1_pressed = bool(l1)
            self.l2_pressed = bool(l2)
            self.up_pressed = bool(up)
            self.down_pressed = bool(down)

    def _get_yaw(self, quat):
        w, x, y, z = quat
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def _sport_state_callback(self, msg: SportModeState_):
        with self.lock:
            # Always update obstacle distance for safety
            self.front_obstacle_dist = msg.range_obstacle[0]
            
            # Only use sport mode for position if UDP localizer is NOT active
            if self.received_udp:
                return
                
            self.current_pos = np.array([msg.position[0], msg.position[1]])
            self.current_yaw = self._get_yaw(msg.imu_state.quaternion)
            
            if self.is_recording:
                self._add_breadcrumb()

    def _add_breadcrumb(self):
        # Should be called with lock held
        if not self.recorded_path:
            self.recorded_path.append(np.array([self.current_pos[0], self.current_pos[1], self.current_yaw]))
            print(f"[REC] Start Point: {self.recorded_path[-1]}")
            return

        last = self.recorded_path[-1]
        dist = np.linalg.norm(self.current_pos - last[:2])
        
        # Calculate yaw difference
        yaw_err = self.current_yaw - last[2]
        while yaw_err > np.pi: yaw_err -= 2*np.pi
        while yaw_err < -np.pi: yaw_err += 2*np.pi
        yaw_diff = abs(yaw_err)

        # Trigger on 0.2m movement or 0.1 rad rotation
        if dist > 0.2 or yaw_diff > 0.1:
            self.recorded_path.append(np.array([self.current_pos[0], self.current_pos[1], self.current_yaw]))
            if len(self.recorded_path) % 10 == 0:
                print(f"[REC] Captured {len(self.recorded_path)} points...")

    def _toggle_recording(self):
        self.is_recording = not self.is_recording
        if self.is_recording:
            self.recorded_path = []
            self.is_playing = False
            print("\n>>> RECORDING STARTED (Dist: 0.2m, Yaw: 0.1rad)")
        else:
            print(f"\n>>> RECORDING STOPPED. Saved {len(self.recorded_path)} breadcrumbs.")

    def _start_playback(self):
        if self.is_playing:
            self.is_playing = False
            print("\n>>> PLAYBACK STOPPED")
            return

        if not self.recorded_path:
            print("\n!!! NO TRAIL RECORDED")
            return
        
        self.is_recording = False
        self.is_playing = True
        print(f"\n>>> PLAYBACK STARTED. Path: {len(self.recorded_path)} points.")
        threading.Thread(target=self._playback_loop, daemon=True).start()

    def _playback_loop(self):
        # 1. Path Setup
        path = np.array(self.recorded_path)
        path_len = len(path)
        if path_len < 3:
            print(">>> ERROR: Path too short.")
            self.is_playing = False
            return

        # Controller Parameters
        LOOKAHEAD_DIST = 0.5   
        MAX_VEL = 0.5          
        MAX_YAW_VEL = 1.0      
        FINAL_DIST_TOL = 0.20  
        FINAL_YAW_TOL = 0.15   
        
        # We start from the absolute beginning of the trail
        last_index = 0
        last_debug = 0
        has_started_trail = False
        
        print(f">>> STARTING WAYPOINT NAV: {path_len} points")
        print(f">>> Start Position: {path[0, :2]}")
        print(f">>> Final Destination: {path[-1, :2]}")
        if self.received_udp:
            print(">>> Navigation Source: High-Precision Localizer (UDP)")
        else:
            print(">>> WARNING: Using SLAM-less Fallback (DDS). Position drift likely.")

        while self.is_playing:
            with self.lock:
                pos = self.current_pos.copy()
                yaw = self.current_yaw
                obs = self.front_obstacle_dist
            
            # Obstacle Safety
            if obs < 0.7:
                self.loco_client.Move(0, 0, 0)
                time.sleep(0.5)
                continue

            # 2. Strict Start Logic: Must go to Point 0 first
            if not has_started_trail:
                target = path[0]
                dist_to_start = np.linalg.norm(target[:2] - pos)
                if dist_to_start < 0.3:
                    print(">>> REACHED START OF TRAIL. Switching to path following.")
                    has_started_trail = True
                lookahead_idx = 0
            else:
                # 3. Path Following (Sequential search)
                # Search window ahead of current progress
                search_end = min(last_index + 15, path_len)
                remaining_segment = path[last_index:search_end]
                dists = np.linalg.norm(remaining_segment[:, :2] - pos, axis=1)
                closest_in_window = np.argmin(dists)
                last_index = last_index + closest_in_window
                
                # Look ahead for steering
                lookahead_idx = last_index
                for i in range(last_index, path_len):
                    d = np.linalg.norm(path[i, :2] - pos)
                    lookahead_idx = i
                    if d > LOOKAHEAD_DIST:
                        break
                target = path[lookahead_idx]

            dist_to_final = np.linalg.norm(path[-1, :2] - pos)
            is_final_approach = has_started_trail and ((last_index > path_len * 0.9) or (lookahead_idx == path_len - 1 and dist_to_final < 0.8))

            # 4. Calculate Steering
            diff = target[:2] - pos
            if is_final_approach and dist_to_final < FINAL_DIST_TOL:
                target_yaw = path[-1, 2] # Final recorded pose
            else:
                target_yaw = np.arctan2(diff[1], diff[0])
            
            yaw_err = target_yaw - yaw
            while yaw_err > np.pi: yaw_err -= 2*np.pi
            while yaw_err < -np.pi: yaw_err += 2*np.pi

            # 5. Mission Termination
            if is_final_approach and dist_to_final < FINAL_DIST_TOL and abs(yaw_err) < FINAL_YAW_TOL:
                print(f">>> SUCCESS: Destination reached.")
                break

            # 6. Velocity Control
            if abs(yaw_err) > 0.8: # Aligning
                vx = 0.0
                vyaw = np.clip(1.8 * yaw_err, -MAX_YAW_VEL, MAX_YAW_VEL)
            else:
                alignment_factor = np.cos(yaw_err)
                if is_final_approach:
                    vx = np.clip(0.6 * dist_to_final, 0.15, MAX_VEL)
                else:
                    vx = np.clip(MAX_VEL * alignment_factor, 0.2, MAX_VEL)
                
                vyaw = np.clip(1.8 * yaw_err, -MAX_YAW_VEL, MAX_YAW_VEL)

            self.loco_client.Move(vx, 0.0, vyaw)
            
            if time.time() - last_debug > 2.0:
                print(f"[NAV] Progress: {last_index}/{path_len} | DistToGoal: {dist_to_final:.2f}m")
                last_debug = time.time()
                
            time.sleep(0.05)

        self.loco_client.Move(0, 0, 0)
        self.is_playing = False
        print(">>> PLAYBACK FINISHED")

    def run(self):
        while True:
            time.sleep(1)

if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    app = BreadcrumbFollower(iface)
    app.run()
