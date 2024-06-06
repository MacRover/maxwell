#ifndef _ENUMS_H
#define _ENUMS_H

#include <cstdint>

enum EMC2305_Reg : uint8_t
{
    FAN_1_TACH_TARGET_HIGH_BYTE = 0x3D,
    FAN_1_TACH_TARGET_LOW_BYTE = 0x3C,
    FAN_1_TACH_READING_HIGH_BYTE = 0x3E,
    FAN_1_TACH_READING_LOW_BYTE = 0x3F,
    FAN_1_CONFIG = 0x32,
    FAN_1_TACH_VALID_COUNT = 0x39
};

enum MIC184_Reg : uint8_t
{
    MIC184_TEMP = 0x00,
    MIC184_CONFIG = 0x01,
    MIC184_T_HYST = 0x02,
    MIC184_T_SET = 0x03,
};

enum MIC184_Zone : uint8_t
{
    MIC184_EXTERNAL = 0x20,
    MIC184_INTERNAL = 0x00
};

#endif