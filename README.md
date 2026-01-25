# LILYGO T3 GPS Tracker

A GPS tracker firmware for the LILYGO TTGO T3 LoRa32 V2.1 board that transmits GPS location data over MQTT to a remote broker. Features a web-based configuration portal, OTA updates, and real-time OLED display.

## Features

- 📡 **GPS Tracking** - U-blox GPS module integration with UBX protocol support
- 📶 **MQTT Transmission** - Sends GPS packets to a configurable MQTT broker with authentication support
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

## Quick Start

### 1. Configure WiFi Credentials

Edit [src/main_tx.cpp](src/main_tx.cpp) and set your WiFi credentials:

```cpp
const char* SSID = "your-wifi-ssid";
const char* PASS = "your-wifi-password";
```
### 2. Access Web Portal

1. After uploading, the device will connect to WiFi
2. Check the serial monitor for the device's IP address or mDNS hostname
3. Open a browser and navigate to:
   - `http://gps-tracker.local` (mDNS)
   - Or the IP address shown in serial monitor

4. Configure the MQTT settings:
   - **Server IP**: IP address of your MQTT broker
   - **Server Port**: MQTT port (default: 1883)
   - **MQTT Username**: Username for authentication (optional)
   - **MQTT Password**: Password for authentication (optional)
   - **Reference Location**: Lat/Lon for distance/bearing calculations

### 3. Run the MQTT Client

On your computer, start the Python MQTT client to receive GPS data:

```bash
# Basic usage (anonymous connection)
python test/mqtt_client.py --ip <broker-ip>

# With authentication
python test/mqtt_client.py --ip <broker-ip> --user <username> --pass <password>

# With custom port and topic
python test/mqtt_client.py --ip <broker-ip> --port 1883 --topic gps/tracker --user <username> --pass <password>
```

**Arguments:**
- `--ip` (required): MQTT broker IP address
- `--port` (optional): MQTT broker port (default: 1883)
- `--topic` (optional): MQTT topic (default: gps/tracker)
- `--user` (optional): MQTT username
- `--pass` (optional): MQTT password

The client will display received GPS packets with:
- Sequence number
- Latitude/Longitude coordinates
- GPS fix status and type
- Number of satellites
- Battery level percentage
## Troubleshooting

**WiFi not connecting:**
- Check SSID and password in main_tx.cpp
- Ensure 2.4GHz WiFi is available (ESP32 doesn't support 5GHz)

**MQTT not connecting:**
- Verify broker IP and port are correct
- Check firewall settings on broker
- Ensure authentication credentials match if required

**GPS not working:**
- Check UART connections to GPS module
- Verify baud rate
- Wait for GPS to acquire satellites (may take several minutes outdoors)

**Web interface not loading:**
- Make sure to upload filesystem with `pio run --target uploadfs`
- Check serial monitor for IP address
- Try mDNS hostname: `http://gps-tracker.local`
