#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#define LED_PIN 13
#define SERVO1_PIN 36  
#define SERVO2_PIN 44

Servo servo1;
Servo servo2;

rcl_subscription_t servo1_sub;
rcl_subscription_t servo2_sub;

std_msgs__msg__Float32 servo1_msg;
std_msgs__msg__Float32 servo2_msg;

rclc_executor_t servo_executor;

int servo1_angle_send = 0;
int servo2_angle_send = 0;

void servo1_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    int angle_deg = (int)msg->data;
    servo1_angle_send = constrain(servo1_angle_send + angle_deg * 10, 0, 180);
    servo1.write(servo1_angle_send);
    
}


void servo2_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    int angle_deg = (int)msg->data;
   servo2_angle_send = constrain(servo2_angle_send + angle_deg * 10, 0, 180);
    servo2.write(servo2_angle_send);
}

/**********************************************************************
 * Initialize Subscriptions and Attach Servos
 **********************************************************************/
void servo_setup_subscription(
    rcl_node_t *node,
    rclc_support_t *support,
    rcl_allocator_t *allocator
) {
    // Attach servos to their respective pins
    servo1.attach(SERVO1_PIN);
     if (servo2.attach(SERVO2_PIN)) {
       digitalWrite(LED_PIN, LOW);
       delay(100);
       digitalWrite(LED_PIN, HIGH);
     }
    servo2.attach(SERVO2_PIN);
     if (servo2.attach(SERVO2_PIN)) {
       digitalWrite(LED_PIN, LOW);
       delay(100);
       digitalWrite(LED_PIN, HIGH);

    
     }

    // Subscription for Servo 1
    rclc_subscription_init_default(
        &servo1_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/obc/servo1_angle"
    );

    // Subscription for Servo 2
    rclc_subscription_init_default(
        &servo2_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/obc/servo2_angle"
    );

    // Create executor for handling both subscriptions
    rclc_executor_init(&servo_executor, &support->context, 2, allocator);

    // Add Servo 1 subscription to executor
    rclc_executor_add_subscription(
        &servo_executor,
        &servo1_sub,
        &servo1_msg,
        servo1_callback,
        ON_NEW_DATA
    );

    // Add Servo 2 subscription to executor
    rclc_executor_add_subscription(
        &servo_executor,
        &servo2_sub,
        &servo2_msg,
        servo2_callback,
        ON_NEW_DATA
    );
}

/**********************************************************************
 * Spin Executor
 **********************************************************************/
void servo_spin_executor() {
    rclc_executor_spin_some(&servo_executor, RCL_MS_TO_NS(10));
}

#endif // SERVO_H