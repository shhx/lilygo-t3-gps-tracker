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

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

#define DNS_NAME "gps-tracker"

const float RECEIVER_LAT = 40.7749f;
const float RECEIVER_LON = -3.4194f;
static uint32_t last_tx_time = 0;
static bool ota_enabled = false;
static uint32_t tx_pkt_counter = 0;
static uint32_t ack_pkt_counter = 0;

static double last_distance_m = 0;
static float battery_voltage = 0.0f;

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

float read_battery_voltage() {
    // Battery voltage divider: LilyGo T3 has a 1/2 divider
    int adc_value = analogRead(BATTERY_ADC_PIN);
    // Convert ADC reading to voltage (accounting for divider)
    float voltage = (adc_value / 4095.0) * 3.3 * 2;
    return voltage;
}

uint8_t calculate_battery_percentage(float voltage) {
    // LiPo battery voltage: 3.0V (empty) to 4.2V (full)
    // Linear approximation
    const float MIN_VOLTAGE = 3.0f;
    const float MAX_VOLTAGE = 4.2f;

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
    display.setCursor(0, 0);
    display.print("Batt: ");
    display.print(battery_voltage, 2);
    display.print("V (");
    display.print(calculate_battery_percentage(battery_voltage));
    display.println("%)");

    // WiFi Status
    display.setCursor(0, 10);
    display.print("WiFi: ");
    if (WiFi.status() == WL_CONNECTED) {
        display.println("Connected");
        display.setCursor(0, 20);
        display.println(WiFi.localIP());
    } else {
        display.println("Disconnected");
        display.setCursor(0, 20);
        display.print(SSID);
        display.print(" : ");
        display.println(PASS);
    }

    // GPS Status
    display.setCursor(0, 40);
    display.print("GPS: ");
    if (nav_pvt.flags.gnssFixOK) {
        display.print("Fix OK Sats:");
        display.println(nav_pvt.numSV);
        display.setCursor(0, 50);
        display.print("Lat: ");
        display.println(nav_pvt.lat * 1e-7, 6);
        display.setCursor(0, 58);
        display.print("Lon: ");
        display.println(nav_pvt.lon * 1e-7, 6);
    } else {
        display.print("No Fix Sats:");
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
        battery_voltage = read_battery_voltage();
    }
    update_oled_display();
    PacketGPS_t packet_gps = {
        .lon = nav_pvt.lon,
        .lat = nav_pvt.lat,
        .fix_ok = nav_pvt.flags.gnssFixOK,
        .fix_type = nav_pvt.fixType,
        .sv_num = (uint8_t)(nav_pvt.numSV > 15 ? 15 : nav_pvt.numSV), // limit to 15 for 4 bits
    };
    // if (digitalRead(BUTTON_PIN) == LOW) {
    //     Serial.println("Button pressed, starting OTA updater");
    //     start_ota_updater();
    // }
    return;
    ublox_get_new_data();
    if (ublox_parse_msg(&nav_pvt)) {
        if(nav_pvt.flags.gnssFixOK) {
            last_distance_m = haversine(RECEIVER_LAT, RECEIVER_LON,
                                        nav_pvt.lat * 1e-7, nav_pvt.lon * 1e-7);
        }
    }
}
