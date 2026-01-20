#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Preferences.h>

#define CONFIG_NAMESPACE "gps_tracker"
#define DEFAULT_SERVER_IP "192.168.1.100"
#define DEFAULT_SERVER_PORT 5000
#define DEFAULT_RECEIVER_LAT 40.439f
#define DEFAULT_RECEIVER_LON -3.6194f
#define DEFAULT_MQTT_USER ""
#define DEFAULT_MQTT_PASS ""

struct Config {
    char server_ip[16];
    uint16_t server_port;
    float receiver_lat;
    float receiver_lon;
    char mqtt_user[32];
    char mqtt_pass[64];
};

class ConfigManager {
public:
    ConfigManager();

    // Initialize and load config from flash
    bool begin();

    // Save configuration to flash
    bool save();

    // Load configuration from flash
    bool load();

    // Reset to default values
    void setDefaults();

    // Getters
    const char* getServerIP() const { return config_.server_ip; }
    uint16_t getServerPort() const { return config_.server_port; }
    float getReceiverLat() const { return config_.receiver_lat; }
    float getReceiverLon() const { return config_.receiver_lon; }
    const char* getMqttUser() const { return config_.mqtt_user; }
    const char* getMqttPass() const { return config_.mqtt_pass; }

    // Setters
    void setServerIP(const char* ip);
    void setServerPort(uint16_t port);
    void setReceiverLat(float lat);
    void setReceiverLon(float lon);
    void setMqttUser(const char* user);
    void setMqttPass(const char* pass);

    // Get full config struct for JSON serialization
    const Config& getConfig() const { return config_; }

private:
    Preferences prefs_;
    Config config_;
};

#endif // CONFIG_H
