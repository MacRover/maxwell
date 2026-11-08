#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR 0x40
#define SERVOMIN 150
#define SERVOMAX 600

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

// Tell the compiler these exist, but define them in the .cpp
extern Adafruit_PWMServoDriver pwm; 
extern rcl_subscription_t servo1_sub, servo2_sub, servo3_sub;
extern std_msgs__msg__Float32 servo1_msg, servo2_msg, servo3_msg;
extern rclc_executor_t servo_executor;

extern int servo1_angle_send;
extern int servo2_angle_send;
extern int servo3_angle_send;

void obc_setup_servo_hardware();
bool servo_setup_subscription(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void setServoAngle(uint8_t channel);
void servo_spin_executor();

#endif