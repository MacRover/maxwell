#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>      
#include <std_msgs/msg/int32.h>
#include "enums.h"           

// --- LED INDICATOR PINS ---
#define PIN_LED_RED 2   
#define PIN_LED_GREEN 3 
#define PIN_LED_BLUE 4  

// --- GLOBAL VARIABLES ---
extern rcl_subscription_t led_sub;
extern std_msgs__msg__Int32 led_msg;
extern rclc_executor_t led_executor;
extern volatile LED_States current_led_state; 

// --- FUNCTION PROTOTYPES ---
void LED_setup();
bool led_setup_subscription(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void led_subscription_callback(const void * msgin);
void LED_SM();

#endif