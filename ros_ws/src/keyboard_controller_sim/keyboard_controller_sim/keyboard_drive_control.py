#! /usr/bin/env python
from math import pi
import rclpy
from rclpy.node import Node

import sys, termios, tty, os, time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class KeyboardController(Node):
    def __init__(self):
        super().__init__("keyboard_controller")
        self.publisher_swerve_ = self.create_publisher(
            Float64MultiArray, "/drive_module_steering_angle_controller/commands", 10
        )
        self.publisher_wheel_ = self.create_publisher(
            Float64MultiArray, "/drive_module_velocity_controller/commands", 10
        )
        self.swerve_msg = Float64MultiArray()
        self.swerve_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.wheel_msg = Float64MultiArray()
        self.wheel_msg.data = [0.0, 0.0, 0.0, 0.0]

    def run_command(self, command):
        if command == "w":
            print("Moving forward")
            self.swerve_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_msg.data = [i + 0.05 for i in self.wheel_msg.data]
        elif command == "s":
            print("Moving backward")
            self.swerve_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_msg.data = [i - 0.05 for i in self.wheel_msg.data]
        elif command == "d":
            print("Moving right")
            self.swerve_msg.data = [-1.57, -1.57, -1.57, -1.57]
            self.wheel_msg.data = [i + 0.05 for i in self.wheel_msg.data]
        elif command == "a":
            print("Moving left")
            self.swerve_msg.data = [1.57, 1.57, 1.57, 1.57]
            self.wheel_msg.data = [i + 0.05 for i in self.wheel_msg.data]
        elif command == "i":
            print("Spinning front wheel left")
            self.swerve_msg.data = [1.57, 1.57, 0.0, 0.0]
            self.wheel_msg.data = [
                self.wheel_msg.data[0] + 0.5,
                self.wheel_msg.data[1] + 0.5,
                0.0,
                0.0,
            ]
        elif command == "o":
            print("Spinning front wheel right")
            self.swerve_msg.data = [-1.57, -1.57, 0.0, 0.0]
            self.wheel_msg.data = [
                self.wheel_msg.data[0] + 0.5,
                self.wheel_msg.data[1] + 0.5,
                0.0,
                0.0,
            ]
        elif ord(command) == 32:
            print("Stopping")
            self.swerve_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_msg.data = [0.0, 0.0, 0.0, 0.0]
        else:
            return

        self.publisher_swerve_.publish(self.swerve_msg)
        self.publisher_wheel_.publish(self.wheel_msg)


def getch():  # this function is used to get key input from user in the terminal
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    rclpy.init(args=args)
    kc_node = KeyboardController()

    print(
        "Press w, a, s, d, i, o accordingly to move; space for stopping and Esckey for exiting. Be careful not to hold these keys, as these keys control the acceleration of the bot, and u dont want the bot flying out!!"
    )
    while True:
        char = getch()
        if ord(char) == 27:
            print("Exiting")
            exit(0)
        else:
            kc_node.run_command(char)


if __name__ == "__main__":
    main()
