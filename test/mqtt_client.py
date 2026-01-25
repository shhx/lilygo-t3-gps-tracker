import argparse
import time
from ctypes import Structure, c_int32, c_uint8, sizeof

import paho.mqtt.client as mqtt

TOPIC = "gps/tracker"  # Default, will be updated by argparse

class Packet(Structure):
    """
    C struct representation using ctypes
    Matches the packet.h structure exactly
    """
    _pack_ = 1  # Matches __attribute__((packed))
    _fields_ = [
        ('seq_num', c_uint8),
        ('lon', c_int32),
        ('lat', c_int32),
        ('fix_ok', c_uint8, 1),
        ('fix_type', c_uint8, 3),
        ('sv_num', c_uint8, 4),
        ('battery_level', c_uint8),
    ]

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(TOPIC)

def on_message(client, userdata, msg: mqtt.MQTTMessage):
    try:
        data = msg.payload
        if len(data) != sizeof(Packet):
            print(f"Invalid packet size: expected {sizeof(Packet)}, got {len(data)}")
            return

        packet = Packet.from_buffer_copy(data)

        lon = packet.lon / 1e7
        lat = packet.lat / 1e7
        fix_ok = bool(packet.fix_ok)
        fix_type = packet.fix_type
        sv_num = packet.sv_num
        battery_level = packet.battery_level

        print(f"Received Packet - Seq: {packet.seq_num}, Lat: {lat:.7f}, Lon: {lon:.7f}, "
              f"Fix OK: {fix_ok}, Fix Type: {fix_type}, SVs: {sv_num}, Battery: {battery_level}%")
    except Exception as e:
        print("Error processing message:", e)

def main():
    parser = argparse.ArgumentParser(description='MQTT GPS Tracker Client')
    parser.add_argument('--ip', type=str, required=True, help='MQTT broker IP address')
    parser.add_argument('--port', type=int, default=1883, help='MQTT broker port (default: 1883)')
    parser.add_argument('--topic', type=str, default='gps/tracker', help='MQTT topic (default: gps/tracker)')
    parser.add_argument('--user', type=str, default=None, help='MQTT username (optional)')
    parser.add_argument('--pass', dest='password', type=str, default=None, help='MQTT password (optional)')

    args = parser.parse_args()

    client = mqtt.Client(client_id="pc-client")

    if args.user and args.password:
        client.username_pw_set(args.user, args.password)
        print(f"Connecting with authentication as user: {args.user}")
    else:
        print("Connecting without authentication")

    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {args.ip}:{args.port}, topic: {args.topic}")
    client.connect(args.ip, args.port, keepalive=60)

    # Update the global topic for the on_connect callback
    global TOPIC
    TOPIC = args.topic

    client.loop_start()

    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        print("\nDisconnecting...")

    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    main()
