#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <SPI.h>
#include "ArduinoOTA.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "OTA.h"
#include "packet.h"
#include "pins.h"
#include "ublox_protocol.h"
#include "udp_sender.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

#define DNS_NAME "gps-tracker"

const float RECEIVER_LAT = 40.439f;
const float RECEIVER_LON = -3.6194f;
static uint32_t last_tx_time = 0;
static bool ota_enabled = false;
static uint32_t tx_pkt_counter = 0;
static uint32_t ack_pkt_counter = 0;

// UDP Server Configuration
const char* UDP_SERVER_IP = "192.168.1.100";  // Change this to your server IP
const uint16_t UDP_SERVER_PORT = 5000;         // Change this to your server port
UDPSender udpSender;

static double last_distance_m = 0;
static double last_bearing_deg = 0;
static float battery_voltage = 0.0f;
static float filtered_battery_voltage = 0.0f;
const float BATTERY_FILTER_ALPHA = 0.01f; // Low-pass filter coefficient (0.1 to 0.5 recommended)

wl_status_t last_wifi_status = WL_DISCONNECTED;

ubx_nav_pvt_t nav_pvt;

AsyncWebServer server(80);
const char* SSID = "red";
const char* PASS = "12345678";
const char* ap_ssid = "Radio-TX";
const char* ap_password = "12345678";

void start_ota_updater(void) {
    setupOTA("radiotx");
    ota_enabled = true;
}

void setupFS() {
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed");
        ESP.restart();
    }
}

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    // WiFi.softAP(ap_ssid, ap_password);  // Start the access point
    WiFi.begin(SSID, PASS);
    Serial.print("Connecting to WiFi");
    if (!MDNS.begin(DNS_NAME)) {
        Serial.println("Error initiating mDNS");
    } else {
        Serial.print("mDNS started: ");
        Serial.print(DNS_NAME);
        Serial.println(".local");
        MDNS.addService("http", "tcp", 80);
    }
}

// Haversine formula for accurate great-circle distance
// Input: coordinates in degrees, Output: distance in meters
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R_EARTH = 6371000.0; // Earth's mean radius in meters
    double phi1 = radians(lat1), phi2 = radians(lat2);
    double dphi = radians(lat2 - lat1);
    double dlambda = radians(lon2 - lon1);
    double a = sin(dphi / 2.0) * sin(dphi / 2.0) +
               cos(phi1) * cos(phi2) * sin(dlambda / 2.0) * sin(dlambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R_EARTH * c;
}

// Forward azimuth (initial bearing) using spherical trigonometry
// Output: bearing in degrees (0-360, where 0=North, 90=East, 180=South, 270=West)
double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = radians(lat1);
    double phi2 = radians(lat2);
    double dlambda = radians(lon2 - lon1);

    double x = sin(dlambda) * cos(phi2);
    double y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlambda);

    double theta = atan2(x, y);
    double bearing = fmod(degrees(theta) + 360.0, 360.0); // Normalize to 0-360
    return bearing;
}

float read_battery_voltage() {
    // Battery voltage divider: LilyGo T3 has a 1/2 divider
    int adc_value = analogRead(BATTERY_ADC_PIN);
    const float REF_VOLTAGE = 3.26f;  // Measured reference voltage
    const float DIVIDER_RATIO = 2.0f; // Voltage divider ratio
    const float CALIBRATION_FACTOR = 1.07f; // Calibration factor (adjust based on your measurements)
    float voltage = (adc_value / 4095.0) * REF_VOLTAGE * DIVIDER_RATIO * CALIBRATION_FACTOR;
    return voltage;
}

float get_filtered_battery_voltage(float raw_voltage) {
    // Low-pass filter for smoothing battery voltage readings
    // Reduces noise from ADC readings
    filtered_battery_voltage = (BATTERY_FILTER_ALPHA * raw_voltage) +
                               ((1.0f - BATTERY_FILTER_ALPHA) * filtered_battery_voltage);
    return filtered_battery_voltage;
}

uint8_t calculate_battery_percentage(float voltage) {
    // Linear approximation
    const float MIN_VOLTAGE = 3.4f;
    const float MAX_VOLTAGE = 4.14f;

    if (voltage <= MIN_VOLTAGE) {
        return 0;
    } else if (voltage >= MAX_VOLTAGE) {
        return 100;
    } else {
        return (uint8_t)(((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");

    Wire.begin(OLED_SDA, OLED_SCL);

    if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED initialized");
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("GPS Tracker TX");
        display.display();
    } else {
        Serial.println("OLED init failed");
    }

    setupWiFi();
    pinMode(LED_PIN, OUTPUT);
    pinMode(BATTERY_ADC_PIN, INPUT);
    analogReadResolution(12);

    ublox_init();
    last_tx_time = millis();
    battery_voltage = read_battery_voltage();
    filtered_battery_voltage = battery_voltage; // Initialize filter with first reading

    udpSender.begin(UDP_SERVER_IP, UDP_SERVER_PORT);
}

void handle_wifi() {
    wl_status_t wifi_status = WiFi.status();
    if (wifi_status == WL_CONNECTED && last_wifi_status != WL_CONNECTED) {
        Serial.print("Wifi connected! IP address: ");
        Serial.println(WiFi.localIP());
    } else if (last_wifi_status == WL_CONNECTED && wifi_status != WL_CONNECTED) {
        Serial.println("Wifi disconnected!!!");
    }
    last_wifi_status = wifi_status;
}

void update_oled_display() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Battery Status
    display.setCursor(55, 0);
    display.print("Bat:");
    display.print(filtered_battery_voltage, 2);
    display.print("V ");
    display.print(calculate_battery_percentage(filtered_battery_voltage));
    display.println("%");

    // WiFi Status
    display.setCursor(0, 0);
    display.print("WiFi: ");
    if (WiFi.status() == WL_CONNECTED) {
        display.println("Yes");
        display.setCursor(0, 10);
        display.println(WiFi.localIP());
    } else {
        display.println("No");
        display.setCursor(0, 10);
        display.print(SSID);
        display.print(" : ");
        display.println(PASS);
    }

    // GPS Status
    display.setCursor(0, 25);
    display.print("GPS: ");
    if (nav_pvt.flags.gnssFixOK) {
        display.print("OK Sats:");
        display.println(nav_pvt.numSV);
        display.setCursor(0, 35);
        display.print("Lat: ");
        display.println(nav_pvt.lat * 1e-7, 7);
        display.setCursor(0, 45);
        display.print("Lon: ");
        display.println(nav_pvt.lon * 1e-7, 7);
        // Distance and Bearing
        display.setCursor(0, 55);
        display.print("D:");
        display.print(last_distance_m, 0);
        display.print("m ");
        display.print("Brg:");
        display.print(last_bearing_deg, 1);
        display.println((char)247); // degree symbol
    } else {
        display.print("No Sats:");
        display.println(nav_pvt.numSV);
    }

    display.display();
}

void loop() {
    handle_wifi();
    if (ota_enabled) {
        // Stop web server to avoid conflicts during OTA
        server.end();
        ArduinoOTA.handle();
        // If OTA is enabled, we return early to avoid processing other tasks
        return;
    }
    if (millis() - last_tx_time > 1000) {
        static int led_state = LOW;
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
        last_tx_time = millis();
    }
    battery_voltage = read_battery_voltage();
    get_filtered_battery_voltage(battery_voltage);
    update_oled_display();

    // if (digitalRead(BUTTON_PIN) == LOW) {
    //     Serial.println("Button pressed, starting OTA updater");
    //     start_ota_updater();
    // }
    ublox_get_new_data();
    if (ublox_parse_msg(&nav_pvt)) {
        if(nav_pvt.flags.gnssFixOK) {
            double gps_lat = nav_pvt.lat * 1e-7;
            double gps_lon = nav_pvt.lon * 1e-7;
            last_distance_m = haversine_distance(RECEIVER_LAT, RECEIVER_LON, gps_lat, gps_lon);
            last_bearing_deg = calculate_bearing(RECEIVER_LAT, RECEIVER_LON, gps_lat, gps_lon);

            // Send GPS data via UDP when WiFi is available
            if (udpSender.isReady()) {
                Packet_t packet = {
                    .seq_num = (uint8_t)tx_pkt_counter++,
                    .lon = nav_pvt.lon,
                    .lat = nav_pvt.lat,
                    .fix_ok = nav_pvt.flags.gnssFixOK,
                    .fix_type = nav_pvt.fixType,
                    .sv_num = (uint8_t)(nav_pvt.numSV > 15 ? 15 : nav_pvt.numSV), // limit to 15 for 4 bits
                    .battery_level = calculate_battery_percentage(filtered_battery_voltage)
                };

                if (udpSender.sendPacket(packet)) {
                    Serial.println("GPS data sent via UDP");
                } else {
                    Serial.println("Failed to send GPS data via UDP");
                }
            }
        }
    }
}
