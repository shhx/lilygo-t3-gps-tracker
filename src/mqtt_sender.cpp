#include "mqtt_sender.h"

MQTTSender::MQTTSender()
    : mqtt_client_(wifi_client_),
      server_port_(1883),
      topic_("gps/tracker"),
      configured_(false),
      last_reconnect_attempt_(0) {
}

void MQTTSender::begin(const char* server_ip, uint16_t server_port) {
    server_ip_ = String(server_ip);
    server_port_ = server_port;
    mqtt_client_.setServer(server_ip, server_port);
    configured_ = true;

    Serial.print("MQTT Sender configured: ");
    Serial.print(server_ip);
    Serial.print(":");
    Serial.println(server_port);
}

void MQTTSender::setAuth(const char* username, const char* password) {
    username_ = String(username);
    password_ = String(password);

    if (username_.length() > 0) {
        Serial.println("MQTT authentication enabled");
    }
}

void MQTTSender::setTopic(const char* topic) {
    topic_ = String(topic);
}

bool MQTTSender::reconnect() {
    if (!configured_ || WiFi.status() != WL_CONNECTED) {
        return false;
    }

    uint32_t now = millis();
    if (now - last_reconnect_attempt_ < 1000) {
        return false;
    }
    last_reconnect_attempt_ = now;

    Serial.print("Attempting MQTT connection...");

    // Create a client ID
    String clientId = "ESP32-GPS";

    // Attempt to connect with or without authentication
    bool connected = false;
    if (username_.length() > 0) {
        connected = mqtt_client_.connect(clientId.c_str(), username_.c_str(), password_.c_str());
    } else {
        connected = mqtt_client_.connect(clientId.c_str());
    }

    if (connected) {
        Serial.println("connected");
        return true;
    } else {
        Serial.print("failed, rc=");
        Serial.println(mqtt_client_.state());
        return false;
    }
}

void MQTTSender::loop() {
    if (!configured_) {
        return;
    }

    if (!mqtt_client_.connected()) {
        reconnect();
    } else {
        mqtt_client_.loop();
    }
}

bool MQTTSender::isReady() {
    return configured_ && mqtt_client_.connected();
}

bool MQTTSender::sendPacket(const Packet_t& packet) {
    if (!isReady()) {
        return false;
    }

    // Publish binary packet data
    bool result = mqtt_client_.publish(topic_.c_str(), (const uint8_t*)&packet, sizeof(Packet_t), false);

    if (!result) {
        Serial.println("MQTT: Failed to publish packet");
    }

    return result;
}
