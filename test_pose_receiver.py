#!/usr/bin/env python3
"""
Quick test to see if pose packets are being received on port 9871.
Run this on the robot while the localizer is running.
"""

import socket
import struct
import time

def test_pose_receiver():
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 9871))
    sock.settimeout(1.0)
    
    print("Listening for pose packets on port 9871...")
    print("Press Ctrl+C to stop")
    print()
    
    packet_count = 0
    last_print = time.time()
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                
                if len(data) >= 36:
                    # Parse packet: timestamp(8) + pos(12) + quat(16)
                    timestamp = struct.unpack('<Q', data[0:8])[0]
                    pos = struct.unpack('<fff', data[8:20])
                    quat = struct.unpack('<ffff', data[20:36])
                    
                    packet_count += 1
                    
                    # Print every second
                    now = time.time()
                    if now - last_print >= 1.0:
                        print(f"Received {packet_count} packets")
                        print(f"  From: {addr}")
                        print(f"  Timestamp: {timestamp}")
                        print(f"  Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
                        print(f"  Quaternion: w={quat[0]:.3f}, x={quat[1]:.3f}, y={quat[2]:.3f}, z={quat[3]:.3f}")
                        print()
                        last_print = now
                else:
                    print(f"Received packet of wrong size: {len(data)} bytes (expected 36)")
                    
            except socket.timeout:
                if packet_count == 0:
                    print("No packets received yet... (waiting)")
                continue
                
    except KeyboardInterrupt:
        print(f"\nStopped. Total packets received: {packet_count}")
        sock.close()

if __name__ == "__main__":
    test_pose_receiver()
