#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "packet.h"

class UDPSender {
public:
    UDPSender();

    // Configure the UDP server address and port
    void begin(const char* server_ip, uint16_t server_port);

    // Send packet via UDP
    bool sendPacket(const Packet_t& packet);

    // Check if UDP is configured and WiFi is connected
    bool isReady();

private:
    WiFiUDP udp_;
    IPAddress server_ip_;
    uint16_t server_port_;
    bool configured_;
};

#endif // UDP_SENDER_H
