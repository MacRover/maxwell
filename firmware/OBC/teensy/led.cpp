#include "led.h"

// Initialize the variables here
rcl_subscription_t led_sub;
std_msgs__msg__Int32 led_msg;
rclc_executor_t led_executor;

volatile LED_States current_led_state = LED_STATE_OFF;
unsigned long last_flash_time = 0;
bool flash_state = false;

void LED_setup() {
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  LED_SM();
}

bool led_setup_subscription(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
  rcl_ret_t rc = rclc_subscription_init_default(
    &led_sub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "rover_led_state"
  );
  if (rc != RCL_RET_OK) { return false; }

  rc = rclc_executor_init(&led_executor, &support->context, 1, allocator);
  if (rc != RCL_RET_OK) { return false; }

  rc = rclc_executor_add_subscription(
    &led_executor,
    &led_sub,
    &led_msg,
    &led_subscription_callback,
    ON_NEW_DATA
  );
  return rc == RCL_RET_OK;
}
// Subscriber Callback
void led_subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  current_led_state = static_cast<LED_States>(msg->data);
}

// State Machine
void LED_SM() {
  switch (current_led_state) {
    case LED_STATE_AUTO: // Solid Red
      analogWrite(PIN_LED_RED, 255);
      analogWrite(PIN_LED_GREEN, 0);
      analogWrite(PIN_LED_BLUE, 0);
      break;
      
    case LED_STATE_TELEOP: // Solid Blue
      analogWrite(PIN_LED_RED, 0);
      analogWrite(PIN_LED_GREEN, 0);
      analogWrite(PIN_LED_BLUE, 255);
      break;
      
    case LED_STATE_ARRIVED: // Flashing Green
      analogWrite(PIN_LED_RED, 0);
      analogWrite(PIN_LED_BLUE, 0);
      if (millis() - last_flash_time > 500) { 
        last_flash_time = millis();
        flash_state = !flash_state;
        analogWrite(PIN_LED_GREEN, flash_state ? 255 : 0);
      }
      break;
      
    default: // Off / Idle
      analogWrite(PIN_LED_RED, 0);
      analogWrite(PIN_LED_GREEN, 0);
      analogWrite(PIN_LED_BLUE, 0);
      break;
  }
}
