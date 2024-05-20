#ifndef _FANS_H
#define _FANS_H
#include <cstdint>
#include <Wire.h>
#include "enums.h"

#define WIRE Wire1
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

void initializeFan(Fan* fan, uint8_t id) 
{
    if (id > 7) return;
    fan->id = id;
    fan->address = (id > 4) ? U15_ADDR : U14_ADDR;
    fan->offset = 16 * (id % 5);
}

void setFanRPM(Fan* fan, uint16_t speed) 
{
    if (speed < MIN_RPM) speed = MIN_RPM;
    if (speed > MAX_RPM) speed = MAX_RPM;

    uint16_t tachCount = (uint16_t)(7864320 / speed);
    fan->tach_target = speed;
    uint8_t high = (tachCount & 0x1FE0) >> 5;
    uint8_t low = (tachCount & 0x1F) << 3;

    WIRE.beginTransmission(fan->address);
    WIRE.write(FAN_1_TACH_TARGET_HIGH_BYTE + (fan->offset));
    WIRE.write(high);
    WIRE.endTransmission();

    WIRE.beginTransmission(fan->address);
    WIRE.write(FAN_1_TACH_TARGET_LOW_BYTE + (fan->offset));
    WIRE.write(low);
    WIRE.endTransmission();
}

void getFanRPM(Fan* fan)
{
    uint16_t count, hCount, lCount;

    WIRE.beginTransmission(fan->address);
    WIRE.write(FAN_1_TACH_READING_HIGH_BYTE + (fan->offset));
    WIRE.endTransmission();
    WIRE.requestFrom(fan->address, 1);
    hCount = WIRE.read();

    WIRE.beginTransmission(fan->address);
    WIRE.write(FAN_1_TACH_READING_LOW_BYTE + (fan->offset));
    WIRE.endTransmission();
    WIRE.requestFrom(fan->address, 1);
    lCount = WIRE.read();

    count = (hCount << 5) | (lCount >> 3);
    fan->tach_reading = (uint16_t)(7864320 / count);
    // Serial.print("RPM: ");
    // Serial.println(fan->tach_reading);
}

void enableFanControl(Fan* fan)
{
    WIRE.beginTransmission(fan->address);
    WIRE.write(FAN_1_CONFIG + (fan->offset));
    WIRE.write(0x2B | (1 << 7));
    WIRE.endTransmission();

    WIRE.beginTransmission(fan->address);
    WIRE.write(FAN_1_TACH_VALID_COUNT + (fan->offset));
    WIRE.write(0xF6);
    WIRE.endTransmission();
}

#endif