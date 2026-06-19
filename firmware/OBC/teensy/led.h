#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

// --- LED INDICATOR PINS ---
#define PIN_LED_RED 2   
#define PIN_LED_GREEN 3 
#define PIN_LED_BLUE 4  

// --- LED STATES ---
#define LED_STATE_OFF 0
#define LED_STATE_AUTO 1
#define LED_STATE_TELEOP 2
#define LED_STATE_ARRIVED 3

// --- GLOBAL VARIABLES (Extern tells the compiler they exist elsewhere) ---
extern rcl_subscription_t led_sub;
extern std_msgs__msg__Int32 led_msg;
extern rclc_executor_t led_executor;
extern volatile int current_led_state;

// --- FUNCTION PROTOTYPES ---
void LED_setup();
bool led_setup_subscription(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void led_subscription_callback(const void * msgin);
void LED_SM();

#endif