#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <Adafruit_PWMServoDriver.h>


#define LED_PIN 13
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 37
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}


Servo servo1, servo2, servo3;

rcl_subscription_t servo1_sub, servo2_sub, servo3_sub;

std_msgs__msg__Float32 servo1_msg, servo2_msg, servo3_msg;

rclc_executor_t servo_executor;

extern int servo1_angle_send;
extern int servo2_angle_send;
extern int servo3_angle_send;


void servo1_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    int angle_deg = (int)msg->data;
    servo1_angle_send = constrain(angle_deg, 0, 180);
}


void servo2_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    int angle_deg = (int)msg->data;
   servo2_angle_send = constrain(angle_deg, 0, 180);
}

void servo3_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    int angle_deg = (int)msg->data;
   servo3_angle_send = constrain(angle_deg, 0, 180);
}

bool servo_setup_subscription(
    rcl_node_t *node,
    rclc_support_t *support,
    rcl_allocator_t *allocator
) {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
     

    RCCHECK(rclc_subscription_init_default(
        &servo1_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/obc/servo1_angle"
    ));

 
    RCCHECK(rclc_subscription_init_default(
        &servo2_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/obc/servo2_angle"
    ));
    RCCHECK(rclc_subscription_init_default(
        &servo3_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/obc/servo3_angle"
    ));


    RCCHECK(rclc_executor_init(&servo_executor, &support->context, 3, allocator));

 
    RCCHECK(rclc_executor_add_subscription(
        &servo_executor,
        &servo1_sub,
        &servo1_msg,
        servo1_callback,
        ON_NEW_DATA
    ));

    
    RCCHECK(rclc_executor_add_subscription(
        &servo_executor,
        &servo2_sub,
        &servo2_msg,
        servo2_callback,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_subscription(
        &servo_executor,
        &servo3_sub,
        &servo3_msg,
        servo3_callback,
        ON_NEW_DATA
    ));
    return true;
}

void servo_spin_executor() {
    rclc_executor_spin_some(&servo_executor, RCL_MS_TO_NS(1));
    setServoAngle(0); // Changed because we are using PWM
}

#endif 