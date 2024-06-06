#ifndef _FANS_H
#define _FANS_H
#include <cstdint>
#include "i2c.h"
#include "enums.h"

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

    write_i2c_8bit(
        fan->address, 
        FAN_1_TACH_TARGET_HIGH_BYTE + (fan->offset), 
        high);
    
    write_i2c_8bit(
        fan->address, 
        FAN_1_TACH_TARGET_LOW_BYTE + (fan->offset), 
        low);
}

void getFanRPM(Fan* fan)
{
    uint8_t hCount, lCount;
    uint16_t count;

    read_i2c_8bit(
        fan->address,
        FAN_1_TACH_READING_HIGH_BYTE + (fan->offset),
        &hCount);

    read_i2c_8bit(
        fan->address,
        FAN_1_TACH_READING_LOW_BYTE + (fan->offset),
        &lCount);

    count = ((uint16_t)hCount << 5) | (lCount >> 3);
    fan->tach_reading = (uint16_t)(7864320 / count);
    // Serial.print("RPM: ");
    // Serial.println(fan->tach_reading);
}

void enableFanControl(Fan* fan)
{
    write_i2c_8bit(
        fan->address, 
        FAN_1_CONFIG + (fan->offset), 
        0x2B | (1 << 7));
        
    write_i2c_8bit(
        fan->address,
        FAN_1_TACH_VALID_COUNT + (fan->offset),
        0xF6);
}

#endif