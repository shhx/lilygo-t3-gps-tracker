#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <SPI.h>
#include "ArduinoOTA.h"
#include <U8g2lib.h>
#include <Wire.h>

#include "OTA.h"
#include "packet.h"
#include "pins.h"
#include "ublox_protocol.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);

#define DNS_NAME "gps"

const float RECEIVER_LAT = 40.7749f;
const float RECEIVER_LON = -3.4194f;
static uint32_t last_tx_time = 0;
static bool ota_enabled = false;
static uint32_t tx_pkt_counter = 0;
static uint32_t ack_pkt_counter = 0;
static int16_t last_rssi = 0;

static uint32_t last_ack_received_time = 0;
static uint32_t last_pkt_sent_time = 0;
static double last_distance_m = 0;

// History buffers for plotting (adjust size to balance memory and history depth)
#define STATS_HISTORY_LEN 120 // Save last 120 samples (~4 minutes at 2s updates)
static int16_t rssi_history[STATS_HISTORY_LEN] = {0};
static double distance_history[STATS_HISTORY_LEN] = {0};
static int stats_head = 0;
static bool stats_buffer_full = false;

wl_status_t last_wifi_status = WL_DISCONNECTED;

ubx_nav_pvt_t nav_pvt;

AsyncWebServer server(80);
const char* SSID = "hola";
const char* PASS = "hola1234";
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
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_5dBm);
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

double haversine(float lat1, float lon1, float lat2, float lon2) {
    // all in degrees, returns meters
    constexpr double R_EARTH = 6371000.0;
    double phi1 = radians(lat1), phi2 = radians(lat2);
    double dphi = radians(lat2-lat1);
    double dlambda = radians(lon2-lon1);
    double a = sin(dphi/2.0)*sin(dphi/2.0) +
               cos(phi1)*cos(phi2)*sin(dlambda/2.0)*sin(dlambda/2.0);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return R_EARTH * c;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");

    // Initialize I2C for OLED
    Wire.begin(OLED_SDA, OLED_SCL);

    // Initialize OLED display
    if (!u8g2.begin()) {
        Serial.println("OLED init failed!");
    } else {
        Serial.println("OLED initialized");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr(0, 10, "Initializing...");
        u8g2.sendBuffer();
    }

    // setupFS();
    setupWiFi();
    pinMode(LED_PIN, OUTPUT);

    ublox_init();
    last_tx_time = millis();
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

void update_history_buffers(int16_t rssi, double distance) {
    rssi_history[stats_head] = rssi;
    distance_history[stats_head] = distance;
    stats_head++;
    if (stats_head >= STATS_HISTORY_LEN) {
        stats_head = 0;
        stats_buffer_full = true;
    }
}

void update_oled_display() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);

    // Title
    u8g2.drawStr(0, 10, "GPS Tracker TX");
    u8g2.drawLine(0, 12, 128, 12);

    // WiFi Status
    u8g2.drawStr(0, 25, "WiFi:");
    if (WiFi.status() == WL_CONNECTED) {
        u8g2.drawStr(40, 25, "Connected");
        char ip_str[20];
        snprintf(ip_str, sizeof(ip_str), "%s", WiFi.localIP().toString().c_str());
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(0, 35, ip_str);
        u8g2.setFont(u8g2_font_6x10_tr);
    } else {
        u8g2.drawStr(40, 25, "Disconnected");
    }

    // GPS Status
    u8g2.drawStr(0, 48, "GPS:");
    if (nav_pvt.flags.gnssFixOK) {
        u8g2.drawStr(40, 48, "Fix OK");
        char gps_info[30];
        snprintf(gps_info, sizeof(gps_info), "Sats: %d", nav_pvt.numSV);
        u8g2.drawStr(0, 58, gps_info);
    } else {
        u8g2.drawStr(40, 48, "No Fix");
        char gps_info[30];
        snprintf(gps_info, sizeof(gps_info), "Sats: %d", nav_pvt.numSV);
        u8g2.drawStr(0, 58, gps_info);
    }

    u8g2.sendBuffer();
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
        update_oled_display();
    }
    PacketGPS_t packet_gps = {
        .lon = nav_pvt.lon,
        .lat = nav_pvt.lat,
        .fix_ok = nav_pvt.flags.gnssFixOK,
        .fix_type = nav_pvt.fixType,
        .sv_num = (uint8_t)(nav_pvt.numSV > 15 ? 15 : nav_pvt.numSV), // limit to 15 for 4 bits
    };
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed, starting OTA updater");
        start_ota_updater();
    }
    ublox_get_new_data();
    if (ublox_parse_msg(&nav_pvt)) {
        if(nav_pvt.flags.gnssFixOK) {
            last_distance_m = haversine(RECEIVER_LAT, RECEIVER_LON,
                                        nav_pvt.lat * 1e-7, nav_pvt.lon * 1e-7);
        }
    }
}
