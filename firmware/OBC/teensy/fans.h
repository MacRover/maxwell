#ifndef _FANS_H
#define _FANS_H
#include <cstdint>
#include "i2c.h"
#include "enums.h"

#define U14_ADDR (uint8_t)0x2E
#define U15_ADDR (uint8_t)0x2C

#define MIN_RPM 1000
#define MAX_RPM 3800

#define MIN_TEMP 25
#define MAX_TEMP 35

double speed_index = 3000;

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
    //Serial.print("tachcount: ");
    //Serial.println(tachCount);
    //Serial.print("Set RPM: ");
    //Serial.println(speed);

    write_i2c_8bit(
        fan->address, 
        FAN_1_TACH_TARGET_LOW_BYTE + (fan->offset), 
        low);
    write_i2c_8bit(
        fan->address, 
        FAN_1_TACH_TARGET_HIGH_BYTE + (fan->offset), 
        high);
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
    //Serial.print("Reading RPM: ");
    //Serial.println(fan->tach_reading);
}

void enableFanControl(Fan* fan)
{
    write_i2c_8bit(
        fan->address, 
        FAN_1_CONFIG + (fan->offset), 
        0x3B | (1 << 7));
        
    write_i2c_8bit(
        fan->address,
        FAN_1_TACH_VALID_COUNT + (fan->offset),
        0xF6);
}


// Test-only function
void testPWMControl(Fan *fan)  {
    if (speed_index<MIN_RPM || speed_index>MAX_RPM){
        speed_index =MIN_RPM;
    }
    setFanRPM(fan, speed_index);
    speed_index = speed_index + 0.5;
    //Serial.print("Speed_index: ");
    //Serial.println(speed_index);
    getFanRPM(fan);
}

void TSBControlFAN(Fan* fan, TSB* tsb) {
    float temp = tsb->ambient_temp;

    float fraction = (temp-MIN_TEMP)/(MAX_TEMP-MIN_TEMP);

    float clampedFraction = fraction<0.0f ? 0.0f : fraction;
    clampedFraction = clampedFraction>1.0f ? 1.0f : clampedFraction;

    uint16_t rpm = clampedFraction*(MAX_RPM-MIN_RPM) + MIN_RPM;

    //getFanRPM(fan);
    Serial.print("temp: ");
    Serial.println(temp);
    Serial.print("Speed: ");
    Serial.println(rpm);
    Serial.print("Fraction: ");
    Serial.println(clampedFraction);
    setFanRPM(fan, rpm) ;

}



#endif

