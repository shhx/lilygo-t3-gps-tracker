# LILYGO T3 GPS Tracker

A GPS tracker firmware for the LILYGO TTGO T3 LoRa32 V2.1 board that transmits GPS location data over UDP to a remote server. Features a web-based configuration portal, OTA updates, and real-time OLED display.

## Features

- 📡 **GPS Tracking** - U-blox GPS module integration with UBX protocol support
- 📶 **WiFi UDP Transmission** - Sends GPS packets to a configurable UDP server
- 🌐 **Web Configuration Portal** - Modern web interface for easy setup
- 📊 **OLED Display** - Real-time display of GPS data, battery, and connection status
- 🔋 **Battery Monitoring** - Voltage monitoring with filtered readings
- 📍 **Distance & Bearing** - Calculates distance and bearing to a reference point
- 💾 **Persistent Configuration** - Settings saved to flash memory

## Hardware Requirements

- **LILYGO TTGO T3 LoRa32 V2.1** board
- U-blox GPS module (connected via UART)
- Built-in OLED display (128x64, SSD1306)

## Pin Configuration

See [src/pins.h](src/pins.h) for detailed pin assignments.

### Access Web Portal

1. After uploading, the device will connect to WiFi
2. Check the serial monitor for the device's IP address or mDNS hostname
3. Open a browser and navigate to:
   - `http://gps-tracker.local` (mDNS)
   - Or the IP address shown in serial monitor

4. Configure the UDP server settings:
   - **Server IP**: IP address of your computer/server
   - **Server Port**: UDP port (default: 5000)
   - **Reference Location**: Lat/Lon for distance/bearing calculations

### 5. Run the UDP Server

On your computer, start the Python UDP server to receive GPS data:

```bash
python test/udp_server.py          # Uses default port 5000
python test/udp_server.py 8080     # Use custom port
```

The server will display received GPS packets with:
- Sequence number
- Latitude/Longitude coordinates
- GPS fix status and type
- Number of satellites
- Battery level
