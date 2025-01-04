#ifndef _SERVO_H
#define _SERVO_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t servo1_subscriber;
rcl_subscription_t servo2_subscriber;

std_msgs__msg__Float32 servo1_msg;
std_msgs__msg__Float32 servo2_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void servo1_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  Serial.print("Servo 1 Angle");
  serial.println(msg->data);
}

void servo2_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  Serial.print("Servo 2 Angle");
  serial.println(msg->data);
}


void loop_teensy_subscriber() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

#endif


