#include "udp_sender.h"

UDPSender::UDPSender() : server_port_(0), configured_(false) {
}

void UDPSender::begin(const char* server_ip, uint16_t server_port) {
    if (server_ip_.fromString(server_ip)) {
        server_port_ = server_port;
        configured_ = true;
        Serial.print("UDP Sender configured: ");
        Serial.print(server_ip);
        Serial.print(":");
        Serial.println(server_port);
    } else {
        Serial.println("UDP Sender: Invalid IP address");
        configured_ = false;
    }
}

bool UDPSender::isReady() {
    return configured_ && (WiFi.status() == WL_CONNECTED);
}

bool UDPSender::sendPacket(const Packet_t& packet) {
    if (!isReady()) {
        return false;
    }

    // Begin UDP packet
    if (udp_.beginPacket(server_ip_, server_port_) == 0) {
        Serial.println("UDP: Failed to begin packet");
        return false;
    }

    // Write packet data
    size_t written = udp_.write((const uint8_t*)&packet, sizeof(Packet_t));
    if (written != sizeof(Packet_t)) {
        Serial.print("UDP: Partial write, expected ");
        Serial.print(sizeof(Packet_t));
        Serial.print(" bytes, wrote ");
        Serial.println(written);
        return false;
    }

    // End and send packet
    if (udp_.endPacket() == 0) {
        Serial.println("UDP: Failed to send packet");
        return false;
    }

    return true;
}
