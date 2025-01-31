#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

def servo_callback_servo1(msg):
    print(f"Received servo1 angle: {msg.data}")
def servo_callback_servo2(msg):
    print(f"Received servo2 angle: {msg.data}")

def main():
    rclpy.init(args=None)
    node = Node("servo_angle_subscriber")

    node.create_subscription(Float32, "servo1_angle", servo_callback_servo1, 10)
    node.create_subscription(Float32, "servo2_angle", servo_callback_servo2, 10)

    print("Subscribed to 'servo_keyboard' topic")
    

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

