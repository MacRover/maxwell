#ifndef _TSB_H
#define _TSB_H
#include <cstdint>
#include <Wire.h>
#include "Adafruit_MCP9601.h"
#include "enums.h"

#define WIRE Wire1
#define U16_ADDR (uint8_t)0x70
#define MCP9601_ADDR (uint8_t)0x60
#define MIC184_ADDR (uint8_t)0x48

typedef struct TSB 
{
    uint8_t channel;
    float ambient_temp;
    float thermocouple_temp;
    int32_t adc;

    float local_temp;
    float remote_temp;

} TSB;


void setChannel(TSB* tsb, uint8_t channel_)
{
    tsb->channel = channel_;
}

void _tca9544a(uint8_t b)
{
    WIRE.beginTransmission(U16_ADDR);
    WIRE.write(4 | (b & 3));
    WIRE.endTransmission();
}

float _mic184_read_temp() 
{
    int16_t temp;
    WIRE.beginTransmission(MIC184_ADDR);
    WIRE.write(MIC184_TEMP);
    WIRE.endTransmission();

    WIRE.requestFrom(MIC184_ADDR, 2);
    temp = WIRE.read();
    temp = (temp << 1) | (WIRE.read() >> 7);

    return (float)temp;
}

void _mic184_switch_zone(MIC184_Zone zone)
{
    uint8_t config;
    WIRE.beginTransmission(MIC184_ADDR);
    WIRE.write(MIC184_CONFIG);
    WIRE.endTransmission();

    WIRE.requestFrom(MIC184_ADDR, 1);
    config = WIRE.read();

    WIRE.beginTransmission(MIC184_ADDR);
    WIRE.write(MIC184_CONFIG);
    WIRE.write((config & 0xDF) | zone);
    WIRE.endTransmission();
}

void readTemp(TSB* tsb, Adafruit_MCP9601* mcp)
{
    _tca9544a(tsb->channel);

    tsb->thermocouple_temp = mcp->readThermocouple();

    tsb->ambient_temp = mcp->readAmbient();

    tsb->adc = (mcp->readADC() * 2);

    _mic184_switch_zone(MIC184_INTERNAL);
    tsb->local_temp = _mic184_read_temp();
    
    _mic184_switch_zone(MIC184_EXTERNAL);
    tsb->remote_temp = _mic184_read_temp();
}

#endif