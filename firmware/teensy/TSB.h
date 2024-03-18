#include <cstdint>
#include <Wire.h>

#define WIRE Wire1
#define U16_ADDR (uint8_t)0x70


void _tca9544a(uint8_t b)
{
    WIRE.beginTransmission(U16_ADDR);
    WIRE.write(4 | (b & 3));
    WIRE.endTransmission();
}