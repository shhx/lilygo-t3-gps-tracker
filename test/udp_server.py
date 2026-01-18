#!/usr/bin/env python3
"""
Simple UDP server to receive GPS tracker data
Listens for packets from the LILYGO T3 GPS tracker
"""

import socket
import sys
from ctypes import Structure, c_int32, c_uint8, sizeof
from datetime import datetime


class Packet_t(Structure):
    """
    C struct representation using ctypes
    Matches the packet.h structure exactly
    """
    _pack_ = 1  # Matches __attribute__((packed))
    _fields_ = [
        ('seq_num', c_uint8),
        ('lon', c_int32),
        ('lat', c_int32),
        ('fix_ok', c_uint8, 1),      # 1-bit bitfield
        ('fix_type', c_uint8, 3),    # 3-bit bitfield
        ('sv_num', c_uint8, 4),      # 4-bit bitfield
        ('battery_level', c_uint8),
    ]


class GPSPacket:
    """Wrapper for GPS packet with convenience methods"""

    def __init__(self, data):
        if len(data) != sizeof(Packet_t):
            raise ValueError(f"Expected {sizeof(Packet_t)} bytes, got {len(data)}")

        # Parse binary data directly into ctypes struct
        self.packet = Packet_t.from_buffer_copy(data)

        # Convenience properties with scaling applied
        self.seq_num = self.packet.seq_num
        self.lon = self.packet.lon / 1e7  # Convert from scaled integer
        self.lat = self.packet.lat / 1e7  # Convert from scaled integer
        self.fix_ok = bool(self.packet.fix_ok)
        self.fix_type = self.packet.fix_type
        self.sv_num = self.packet.sv_num
        self.battery_level = self.packet.battery_level
    def __str__(self):
        return (f"Seq: {self.seq_num}, Lat: {self.lat:.7f}, Lon: {self.lon:.7f}, "
                f"Fix OK: {self.fix_ok}, Fix Type: {self.fix_type}, "
                f"SVs: {self.sv_num}, Battery: {self.battery_level}%")

def run_server(host='0.0.0.0', port=5000):
    """Run the UDP server"""
    print(f"Starting UDP server on {host}:{port}")
    print(f"Waiting for GPS tracker packets... (Ctrl+C to stop)\n")

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.settimeout(0.2)

    packet_count = 0

    try:
        while True:
            try:
                # Receive data
                data, addr = sock.recvfrom(1024)
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                try:
                    packet = GPSPacket(data)
                    packet_count += 1
                    print(f"[{timestamp}] From {addr[0]}:{addr[1]} - {packet}")
                except ValueError as e:
                    print(f"[{timestamp}] Invalid packet from {addr[0]}:{addr[1]}: {e}")
                    print(f"  Raw data ({len(data)} bytes): {data.hex()}")
            except socket.timeout:
                # Timeout allows checking for KeyboardInterrupt
                continue

    except KeyboardInterrupt:
        print(f"\n\nServer stopped. Received {packet_count} packets.")
    finally:
        sock.close()


if __name__ == '__main__':
    PORT = 5000
    if len(sys.argv) > 1:
        try:
            PORT = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port: {sys.argv[1]}")
            sys.exit(1)

    run_server(port=PORT)
