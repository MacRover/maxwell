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
#define DEFAULT_SERVO_PIN 36

Servo myservo;
rcl_subscription_t servo_sub;
std_msgs__msg__Float32 servo_msg;
rclc_executor_t servo_executor;

void servo_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

    int angle_deg = (int)msg->data;
    myservo.write(angle_deg);
}

void servo_setup_subscription(
    rcl_node_t *node,
    rclc_support_t *support,
    rcl_allocator_t *allocator,
    const char *topic_name,
    int servo_pin = DEFAULT_SERVO_PIN
) {
    myservo.attach(servo_pin);

    rclc_subscription_init_default(
        &servo_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        topic_name
    );

    rclc_executor_init(&servo_executor, &support->context, 1, allocator);

    rclc_executor_add_subscription(
        &servo_executor,
        &servo_sub,
        &servo_msg,
        servo_callback,
        ON_NEW_DATA
    );
}

void servo_spin_executor() {
    rclc_executor_spin_some(&servo_executor, RCL_MS_TO_NS(10));
}

#endif // SERVO_H
