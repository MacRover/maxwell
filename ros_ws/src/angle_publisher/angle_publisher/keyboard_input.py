#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys, termios, tty, os, time

def getch():  
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init(args=None)
    node = Node("keyboard_angle_controller")
    publisher_servo1 = node.create_publisher(Float32, "servo1_angle", 10)
    publisher_servo2 = node.create_publisher(Float32, "servo2_angle", 10)
    servo_msg1 = Float32()
    servo_msg2 = Float32()

    print(
        """Press w, a, s, d, q, e accordingly to move; space for stopping and Esckey for exiting.
        Be careful not to hold these keys, as these keys control the acceleration of the bot,
        and u dont want the bot flying out!!"""
    )

    enabled = True
    while enabled:
        char = getch()
        if ord(char) == 27:
            print("Exiting")
            servo_msg1.data = 0.0 	
            servo_msg2.data = 0.0
            enabled = False
        elif char == "w":
            print("Servo 1 Accelerated")
            servo_msg1.data += 0.008
        elif char == "s":
            print("Servo 1 Accelerated the opposite direction")
            servo_msg1.data -= 0.008
        elif char == "l":
            print("Servo 2 Accelerated")
            servo_msg2.data += 0.008
        elif char == "k":
            print("Servo 2 Accelerated the opposite direction")
            servo_msg2.data -= 0.008
        
        elif ord(char) == 32:
            print("Stopping")
            servo_msg1.data = 0.0
            servo_msg2.data = 0.0


        print(f"servo_1 : {servo_msg1.data}, servo_2  {servo_msg2.data}")
        publisher_servo1.publish(servo_msg1)
        publisher_servo2.publish(servo_msg2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
