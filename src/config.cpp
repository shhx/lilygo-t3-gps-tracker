#include "config.h"
#include <string.h>

ConfigManager::ConfigManager() {
    setDefaults();
}

bool ConfigManager::begin() {
    if (!prefs_.begin(CONFIG_NAMESPACE, false)) {
        Serial.println("Failed to initialize preferences");
        return false;
    }

    // Check if config exists, if not save defaults
    if (!prefs_.isKey("initialized")) {
        Serial.println("First boot - saving default configuration");
        prefs_.putBool("initialized", true);
        save();
    } else {
        load();
    }

    return true;
}

bool ConfigManager::save() {
    prefs_.putString("server_ip", config_.server_ip);
    prefs_.putUShort("server_port", config_.server_port);
    prefs_.putFloat("receiver_lat", config_.receiver_lat);
    prefs_.putFloat("receiver_lon", config_.receiver_lon);
    prefs_.putString("mqtt_user", config_.mqtt_user);
    prefs_.putString("mqtt_pass", config_.mqtt_pass);

    Serial.println("Configuration saved to flash");
    Serial.printf("  Server: %s:%d\n", config_.server_ip, config_.server_port);
    Serial.printf("  Receiver: %.6f, %.6f\n", config_.receiver_lat, config_.receiver_lon);
    return true;
}

bool ConfigManager::load() {
    String ip = prefs_.getString("server_ip", DEFAULT_SERVER_IP);
    strncpy(config_.server_ip, ip.c_str(), sizeof(config_.server_ip) - 1);
    config_.server_ip[sizeof(config_.server_ip) - 1] = '\0';

    config_.server_port = prefs_.getUShort("server_port", DEFAULT_SERVER_PORT);
    config_.receiver_lat = prefs_.getFloat("receiver_lat", DEFAULT_RECEIVER_LAT);
    config_.receiver_lon = prefs_.getFloat("receiver_lon", DEFAULT_RECEIVER_LON);
    
    String mqtt_user = prefs_.getString("mqtt_user", DEFAULT_MQTT_USER);
    strncpy(config_.mqtt_user, mqtt_user.c_str(), sizeof(config_.mqtt_user) - 1);
    config_.mqtt_user[sizeof(config_.mqtt_user) - 1] = '\0';
    
    String mqtt_pass = prefs_.getString("mqtt_pass", DEFAULT_MQTT_PASS);
    strncpy(config_.mqtt_pass, mqtt_pass.c_str(), sizeof(config_.mqtt_pass) - 1);
    config_.mqtt_pass[sizeof(config_.mqtt_pass) - 1] = '\0';

    Serial.println("Configuration loaded from flash");
    Serial.printf("  Server: %s:%d\n", config_.server_ip, config_.server_port);
    Serial.printf("  Receiver: %.6f, %.6f\n", config_.receiver_lat, config_.receiver_lon);
    Serial.printf("  MQTT User: %s\n", config_.mqtt_user[0] ? "***" : "(none)");

    return true;
}

void ConfigManager::setDefaults() {
    strncpy(config_.server_ip, DEFAULT_SERVER_IP, sizeof(config_.server_ip) - 1);
    config_.server_ip[sizeof(config_.server_ip) - 1] = '\0';
    config_.server_port = DEFAULT_SERVER_PORT;
    config_.receiver_lat = DEFAULT_RECEIVER_LAT;
    config_.receiver_lon = DEFAULT_RECEIVER_LON;
    strncpy(config_.mqtt_user, DEFAULT_MQTT_USER, sizeof(config_.mqtt_user) - 1);
    config_.mqtt_user[sizeof(config_.mqtt_user) - 1] = '\0';
    strncpy(config_.mqtt_pass, DEFAULT_MQTT_PASS, sizeof(config_.mqtt_pass) - 1);
    config_.mqtt_pass[sizeof(config_.mqtt_pass) - 1] = '\0';
}

void ConfigManager::setServerIP(const char* ip) {
    if (ip != nullptr) {
        strncpy(config_.server_ip, ip, sizeof(config_.server_ip) - 1);
        config_.server_ip[sizeof(config_.server_ip) - 1] = '\0';
    }
}

void ConfigManager::setServerPort(uint16_t port) {
    config_.server_port = port;
}

void ConfigManager::setReceiverLat(float lat) {
    config_.receiver_lat = lat;
}

void ConfigManager::setReceiverLon(float lon) {
    config_.receiver_lon = lon;
}

void ConfigManager::setMqttUser(const char* user) {
    if (user != nullptr) {
        strncpy(config_.mqtt_user, user, sizeof(config_.mqtt_user) - 1);
        config_.mqtt_user[sizeof(config_.mqtt_user) - 1] = '\0';
    }
}

void ConfigManager::setMqttPass(const char* pass) {
    if (pass != nullptr) {
        strncpy(config_.mqtt_pass, pass, sizeof(config_.mqtt_pass) - 1);
        config_.mqtt_pass[sizeof(config_.mqtt_pass) - 1] = '\0';
    }
}
