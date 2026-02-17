#include "G1Localizer.hpp"
#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <sstream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    g_running = false;
}

// Simple HTTP Server for Visualization
class WebVisualizer {
public:
    WebVisualizer(g1_localization::G1Localizer* localizer, int port) 
        : localizer_(localizer), port_(port) {}

    void start() {
        server_thread_ = std::thread(&WebVisualizer::run, this);
    }

    void stop() {
        if (server_socket_ != -1) close(server_socket_);
        if (server_thread_.joinable()) server_thread_.join();
    }

private:
    g1_localization::G1Localizer* localizer_;
    int port_;
    int server_socket_ = -1;
    std::thread server_thread_;

    const std::string HTML_PAGE = R"(
<!DOCTYPE html>
<html>
<head>
    <title>G1 LiDAR Debugger</title>
    <style>
        body { margin: 0; background: #000; color: #fff; font-family: monospace; overflow: hidden; }
        #canvas { width: 100vw; height: 100vh; }
        #hud { position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.7); padding: 10px; pointer-events: none; }
    </style>
</head>
<body>
    <div id="hud">
        <h2>LiDAR Alignment View</h2>
        <div>POS: <span id="pos">0.0, 0.0</span></div>
        <div style="color:#888">GRAY: MAPPED WALLS</div>
        <div style="color:#f00">RED: LIVE SCAN</div>
        <div style="color:#0f0">GREEN: ROBOT</div>
    </div>
    <canvas id="canvas"></canvas>
    <script>
        const cvs = document.getElementById('canvas');
        const ctx = cvs.getContext('2d');
        let scale = 50; 
        let offset = {x: window.innerWidth/2, y: window.innerHeight/2};

        function resize() { cvs.width = window.innerWidth; cvs.height = window.innerHeight; }
        window.onresize = resize; resize();

        let drag = false;
        cvs.onmousedown = () => drag = true;
        window.onmouseup = () => drag = false;
        window.onmousemove = e => { if(drag) { offset.x+=e.movementX; offset.y+=e.movementY; }};
        cvs.onwheel = e => scale *= (e.deltaY > 0 ? 0.9 : 1.1);

        async function loop() {
            try {
                let res = await fetch('/data');
                let d = await res.json();
                
                document.getElementById('pos').innerText = d.pos;

                ctx.fillStyle = '#000';
                ctx.fillRect(0,0,cvs.width,cvs.height);

                // Grid
                ctx.strokeStyle = '#222'; ctx.lineWidth = 1;
                for(let i=-10; i<=10; i++) {
                    let x = offset.x + (d.x + i)*scale; // Moving grid relative to robot
                    ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,cvs.height); ctx.stroke();
                }

                // Draw Map (Walls)
                ctx.fillStyle = '#666'; 
                d.map.forEach(p => {
                    ctx.fillRect(offset.x + p[0]*scale - 2, offset.y - p[1]*scale - 2, 4, 4);
                });

                // Draw Robot
                let rx = offset.x + d.x*scale;
                let ry = offset.y - d.y*scale;
                
                ctx.save();
                ctx.translate(rx, ry);
                ctx.rotate(-d.yaw);
                
                // Draw Live Scan (Local Frame transformed)
                ctx.fillStyle = '#f00';
                d.scan.forEach(p => {
                    ctx.fillRect(p[0]*scale - 2, -p[1]*scale - 2, 3, 3);
                });

                // Robot Body
                ctx.fillStyle = '#0f0';
                ctx.beginPath(); ctx.moveTo(10,0); ctx.lineTo(-5,-5); ctx.lineTo(-5,5); ctx.fill();
                
                ctx.restore();

            } catch(e){}
            requestAnimationFrame(loop);
        }
        loop();
    </script>
</body>
</html>
)";

    void run() {
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);
        
        bind(server_socket_, (struct sockaddr*)&addr, sizeof(addr));
        listen(server_socket_, 5);

        // Make non-blocking
        fcntl(server_socket_, F_SETFL, O_NONBLOCK);

        while(g_running) {
            int client = accept(server_socket_, NULL, NULL);
            if(client >= 0) {
                // Read Request (ignore content, just check path)
                char buffer[1024];
                int n = read(client, buffer, 1023);
                if(n > 0) {
                    buffer[n] = 0;
                    std::string req(buffer);
                    
                    std::string response;
                    if(req.find("GET /data") != std::string::npos) {
                        auto state = localizer_->getState();
                        auto map = localizer_->getGlobalMap();
                        auto scan = localizer_->getLatestScan();
                        
                        // JSON Builder
                        std::stringstream ss;
                        ss << "{\"x\":" << state.position.x() 
                           << ",\"y\":" << state.position.y()
                           << ",\"yaw\":" << atan2(2.0f*(state.orientation.w()*state.orientation.z() + state.orientation.x()*state.orientation.y()), 
                                                   1.0f - 2.0f*(state.orientation.y()*state.orientation.y() + state.orientation.z()*state.orientation.z()))
                           << ",\"pos\":\"" << state.position.x() << ", " << state.position.y() << "\""
                           << ",\"map\":[";

                        for(size_t i=0; i<map.size(); ++i) {
                            ss << "[" << map[i].x() << "," << map[i].y() << "]";
                            if(i < map.size()-1) ss << ",";
                        }
                        
                        ss << "], \"scan\":[";
                        
                        for(size_t i=0; i<scan.size(); ++i) {
                            // Transform scan to BODY frame for visualization (since canvas rotates by Robot Yaw)
                            // Actually, the scan in G1Localizer is already in BASE frame relative to map IF we want to show alignment.
                            // BUT our visualizer draws the robot at (0,0) relative to map? No.
                            // Visualizer draws map in absolute frame.
                            // Visualizer transforms ctx to robot position and rotates it.
                            // So if we draw scan in that context, scan points must be in ROBOT BODY frame.
                            // 'latest_scan_2d_' in G1Localizer is in BASE_FRAME (Leveled).
                            // So we can draw them directly inside the robot context!
                            
                            ss << "[" << scan[i].x() << "," << scan[i].y() << "]";
                            if(i < scan.size()-1) ss << ",";
                        }

                        ss << "]}";
                        
                        response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + ss.str();
                    } else {
                        response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" + HTML_PAGE;
                    }
                    send(client, response.c_str(), response.size(), 0);
                }
                close(client);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    
    std::string iface = (argc>=2) ? argv[1] : "eth0";
    std::cout << "Starting G1 Visualizer Node on " << iface << "..." << std::endl;
    std::cout << "Web Interface: http://<IP>:8081" << std::endl;

    g1_localization::G1Localizer localizer(iface, "127.0.0.1"); // Local tracking only
    localizer.start();

    WebVisualizer web(&localizer, 8081);
    web.start();

    while(g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    web.stop();
    localizer.stop();
    return 0;
}
