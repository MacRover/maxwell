#ifndef COOLING_H
#define COOLING_H

#include <Arduino.h>
#include "enums.h"
#include <std_msgs/msg/float32_multi_array.h>

#define USING_TSB
#define USING_FANS

extern TSB_STATES state_TSB;
extern fan_states state_fans;
extern std_msgs__msg__Float32MultiArray tsb_msg;

void update_cooling();

#endif
