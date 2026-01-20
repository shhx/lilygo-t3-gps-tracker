#ifndef MQTT_SENDER_H
#define MQTT_SENDER_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "packet.h"

class MQTTSender {
public:
    MQTTSender();

    // Configure the MQTT broker address and port
    void begin(const char* server_ip, uint16_t server_port);

    // Set MQTT authentication credentials
    void setAuth(const char* username, const char* password);

    // Set MQTT topic (default: "gps/tracker")
    void setTopic(const char* topic);

    // Process MQTT connection and maintain keepalive
    void loop();

    // Send packet via MQTT
    bool sendPacket(const Packet_t& packet);

    // Check if MQTT is configured and connected
    bool isReady();

private:
    WiFiClient wifi_client_;
    PubSubClient mqtt_client_;
    String server_ip_;
    uint16_t server_port_;
    String topic_;
    String username_;
    String password_;
    bool configured_;
    uint32_t last_reconnect_attempt_;

    bool reconnect();
};

#endif // MQTT_SENDER_H
