#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>
#include <std_msgs/msg/u_int8_multi_array.h>

extern std_msgs__msg__UInt8MultiArray health_msg;

void obc_setup_health();
void update_health(uint8_t uros, uint8_t fans, uint8_t tsb, uint8_t lora, uint8_t hydrogen, uint8_t ozone);

#endif