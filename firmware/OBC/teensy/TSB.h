#ifndef _TSB_H
#define _TSB_H
#include <cstdint>
//#include <Adafruit_MCP9601.h>
#include "enums.h"
#include "i2c.h"

#define U16_ADDR (uint8_t)0x70
#define MCP9601_ADDR (uint8_t)0x60
#define MIC184_ADDR (uint8_t)0x48

typedef struct TSB 
{
    uint8_t channel;
    float ambient_temp;
    float thermocouple_temp;
    int32_t adc;

    float mic_temp;

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
    read_i2c_16bit(MIC184_ADDR, MIC184_TEMP, &temp);
    return (float)temp / 255.0f;
}

void mic184_switch_zone(MIC184_Zone zone)
{
    uint8_t config;
    read_i2c_8bit(MIC184_ADDR, MIC184_CONFIG, &config);

    // Set IM HIGH
    config = config | (0x40);
    write_i2c_8bit(MIC184_ADDR, MIC184_CONFIG, config);
    read_i2c_8bit(MIC184_ADDR, MIC184_CONFIG, &config);

    // Set ZONE
    config = (config & 0xDF) | zone;
    write_i2c_8bit(MIC184_ADDR, MIC184_CONFIG, config);

    // Set IM LOW
    read_i2c_8bit(MIC184_ADDR, MIC184_CONFIG, &config);
    write_i2c_8bit(MIC184_ADDR, MIC184_CONFIG, config & 0xBF);
}

void readTemp(TSB* tsb, Adafruit_MCP9601* mcp)
{
    _tca9544a(tsb->channel);

    tsb->thermocouple_temp = mcp->readThermocouple();

    tsb->ambient_temp = mcp->readAmbient();

    tsb->adc = (mcp->readADC() * 2);

    tsb->mic_temp = _mic184_read_temp();
}

#endif