#ifndef _HELPER_H
#define _HELPER_H
#include <cstdint>
#include <Wire.h>

#define WIRE Wire1

void write_i2c_8bit(uint8_t address, uint8_t reg, uint8_t n)
{
    WIRE.beginTransmission(address);
    WIRE.write(reg);
    WIRE.write(n);
    WIRE.endTransmission();
}

void read_i2c_8bit(uint8_t address, uint8_t reg, uint8_t* n)
{
    WIRE.beginTransmission(address);
    WIRE.write(reg);
    WIRE.endTransmission();

    WIRE.requestFrom(address, 1);
    *n = WIRE.read();
}

void read_i2c_16bit(uint8_t address, uint8_t reg, uint16_t* n)
{
    WIRE.beginTransmission(address);
    WIRE.write(reg);
    WIRE.endTransmission();

    WIRE.requestFrom(address, 2);
    *n = WIRE.read() << 8;
    *n = (*n) | ( WIRE.read() );
}


#endif
