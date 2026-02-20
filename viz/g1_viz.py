import socket
import json
import struct
import threading
import time
import asyncio
import websockets
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os

# Configuration
CPP_HOST = '127.0.0.1'
CPP_PORT = 5000
WEB_PORT = 8080
WS_PORT = 8765

latest_data = None
data_lock = threading.Lock()

def tcp_client_thread():
    global latest_data
    while True:
        try:
            print(f"[TCP] Connecting to {CPP_HOST}:{CPP_PORT}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((CPP_HOST, CPP_PORT))
            print("[TCP] Connected!")
            
            while True:
                # Read length (4 bytes)
                len_bytes = sock.recv(4)
                if not len_bytes: break
                
                msg_len = struct.unpack('>I', len_bytes)[0] # Network byte order
                
                # Read data
                data = b''
                while len(data) < msg_len:
                    chunk = sock.recv(min(msg_len - len(data), 4096))
                    if not chunk: break
                    data += chunk
                
                if len(data) != msg_len: break
                
                # Decode JSON
                try:
                    json_str = data.decode('utf-8')
                    # Validate JSON
                    parsed = json.loads(json_str) 
                    
                    with data_lock:
                        latest_data = json_str
                        
                except Exception as e:
                    print(f"[JSON] Error: {e}")
                    
        except Exception as e:
            print(f"[TCP] Connection failed: {e}")
            time.sleep(1)

async def ws_handler(websocket):
    print("[WS] Client connected")
    try:
        while True:
            data_to_send = None
            with data_lock:
                if latest_data:
                    data_to_send = latest_data
            
            if data_to_send:
                await websocket.send(data_to_send)
            
            await asyncio.sleep(0.05) # 20Hz
            
    except websockets.exceptions.ConnectionClosed:
        print("[WS] Client disconnected")

def start_ws_server():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    server = websockets.serve(ws_handler, "0.0.0.0", WS_PORT)
    print(f"[WS] WebSocket server started on port {WS_PORT}")
    loop.run_until_complete(server)
    loop.run_forever()

def start_http_server():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    server = HTTPServer(('0.0.0.0', WEB_PORT), SimpleHTTPRequestHandler)
    print(f"[HTTP] Web server started at http://localhost:{WEB_PORT}")
    server.serve_forever()

if __name__ == "__main__":
    # Start TCP Client
    t1 = threading.Thread(target=tcp_client_thread, daemon=True)
    t1.start()
    
    # Start WebSocket Server
    t2 = threading.Thread(target=start_ws_server, daemon=True)
    t2.start()
    
    # Start HTTP Server (Blocking)
    start_http_server()
