#include <cstdint>
#include <Wire.h>

#define U14_ADDR (uint8_t)0x2E
#define U15_ADDR (uint8_t)0x2C

typedef struct Fan {
    uint8_t id;
    uint8_t address;
    uint8_t offset;
    uint16_t tach_reading;
    uint16_t tach_target;
} Fan;


void initialize_fan(Fan* fan, uint8_t id) 
{
    if (id > 7) return;
    fan->id = id;
    fan->address = (id > 4) ? U15_ADDR : U14_ADDR;
    fan->offset = 16 * (id % 5);
}

void set_fan_rpm(Fan* fan, uint16_t speed) 
{
    fan->tach_target = speed;
    uint8_t reg = 0x3D + (fan->offset);
    uint8_t high = (speed & 0x1FE0) >> 5;
    uint8_t low = (speed & 0x1F) << 3;

    Wire1.beginTransmission(fan->address);
    Wire1.write(reg);
    Wire1.write(high);
    Wire1.endTransmission();

    Wire1.beginTransmission(fan->address);
    Wire1.write(reg - 1);
    Wire1.write(low);
    Wire1.endTransmission();
}

void get_fan_rpm(Fan* fan)
{
    uint16_t count, hCount, lCount;
    uint8_t reg = 0x3E + (fan->offset);

    Wire1.beginTransmission(fan->address);
    Wire1.write(reg + fan->offset);
    Wire1.endTransmission();
    Wire1.requestFrom(fan->address, 1);
    hCount = Wire1.read();

    Wire1.beginTransmission(fan->address);
    Wire1.write(reg + 1 + fan->offset);
    Wire1.endTransmission();
    Wire1.requestFrom(fan->address, 1);
    lCount = Wire1.read();

    count = (hCount << 5) | (lCount >> 3);
    fan->tach_reading = count;
    Serial.print("low byte count: ");
    Serial.print(lCount, BIN);
    Serial.print(" high byte count: ");
    Serial.println(hCount,BIN);
}

void enable_fan_control(Fan* fan)
{
    Wire1.beginTransmission(fan->address);
    Wire1.write(0x32 + (fan->offset));
    Wire1.write(0x2B | (1 << 7));
    Wire1.endTransmission();
}
