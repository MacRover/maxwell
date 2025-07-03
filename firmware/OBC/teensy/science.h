#ifndef _SCIENCE_H
#define _SCIENCE_H

#include "enums.h"

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/Float32.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#define I2C_ADDRESS_HYDROGEN_SENSOR 0x74
#define I2C_ADDRESS_OZONE_SENSOR 0x73

std_msgs__msg__Float32 hydrogen_msg;
std_msgs__msg__Int16 ozone_msg;

DFRobot_GAS_I2C hydrogen_sensor(&Wire1, I2C_ADDRESS_HYDROGEN_SENSOR);
DFRobot_OzoneSensor ozone_sensor(&Wire1);


void update_hydrogen_message(float val)
{
    hydrogen_msg.data = val;
}

void update_ozone_message(int16_t val)
{
    ozone_msg.data = val;
}










#endif