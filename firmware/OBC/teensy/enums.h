#ifndef _ENUMS_H
#define _ENUMS_H

#include <cstdint>


enum UROS_states {
  UROS_INIT,
  UROS_FOUND,
  UROS_OK,
  UROS_ERROR 
};

enum fan_states{
  FANS_INIT,
  FANS_OK,
  FANS_ERROR
};

enum TSB_STATES{
  TSB_INIT,
  TSB_OK,
  TSB_ERROR
};

enum HYDROGEN_STATES{
  HYDROGEN_INIT,
  HYDROGEN_OK,
  HYDROGEN_ERROR
};

enum OZONE_STATES{
  OZONE_INIT,
  OZONE_OK,
  OZONE_ERROR
};

enum LORA_STATES {
  LORA_INIT,            
  LORA_TRANSMIT, 
  LORA_FLAG,   
  LORA_FINISH,          
  LORA_DELAY 
};

enum LED_States {
  LED_STATE_OFF,
  LED_STATE_AUTO,
  LED_STATE_TELEOP,
  LED_STATE_ARRIVED
};

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



extern UROS_states state_UROS;
extern fan_states state_fans;
extern TSB_STATES state_TSB;
extern HYDROGEN_STATES state_hydrogen;
extern OZONE_STATES state_ozone;
extern LORA_STATES state_lora;


#endif