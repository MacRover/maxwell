#include "health.h"

std_msgs__msg__UInt8MultiArray health_msg;
static uint8_t health_data[6];

void obc_setup_health() {
    std_msgs__msg__UInt8MultiArray__init(&health_msg); 
    health_msg.data.capacity = 6;
    health_msg.data.size = 6;
    health_msg.data.data = health_data; 
}

void update_health(uint8_t uros, uint8_t fans, uint8_t tsb, uint8_t lora, uint8_t hydrogen, uint8_t ozone) {
    health_msg.data.data[0] = uros;
    health_msg.data.data[1] = fans;
    health_msg.data.data[2] = tsb;
    health_msg.data.data[3] = lora;
    health_msg.data.data[4] = hydrogen;
    health_msg.data.data[5] = ozone;
}