#ifndef FANS_H
#define FANS_H

#include <cstdint>

#define U14_ADDR (uint8_t)0x2E
#define U15_ADDR (uint8_t)0x2C

#define MIN_RPM 1000
#define MAX_RPM 6400

typedef struct Fan {
    uint8_t id;
    uint8_t address;
    uint8_t offset;
    uint16_t tach_reading;
    uint16_t tach_target;
} Fan;

void initializeFan(Fan* fan, uint8_t id);
void setFanRPM(Fan* fan, uint16_t speed);
void getFanRPM(Fan* fan);
void enableFanControl(Fan* fan);

#endif