#include <Arduino.h>
#ifndef PACKET_H
#define PACKET_H

typedef struct __attribute__((packed)) {
    uint8_t seq_num;
    int32_t lon;
    int32_t lat;
    uint8_t fix_ok : 1;
    uint8_t fix_type: 3;
    uint8_t sv_num: 4;
    uint8_t battery_level; // 0-100%
} Packet_t;

#endif // PACKET_H
