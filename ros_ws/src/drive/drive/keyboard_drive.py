#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from custom_interfaces.msg import SwerveModulePulse

import sys, termios, tty, os, time

from .drive_controller import DriveMode

def getch():  # this function is used to get key input from user in the terminal
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
    node = Node("keyboard_controller")
    publisher = node.create_publisher(Twist, "/drive/cmd_vel", 10)
    pulse_publisher = node.create_publisher(SwerveModulePulse, "/drive/rad_pulses", 10)
    vel_msg = Twist()
    pulse_msg = SwerveModulePulse()

    node.declare_parameter(
        "drive_mode",
        "SWERVE_DRIVE",
        ParameterDescriptor(
            description="Method of Driving (SWERVE_DRIVE | TANK_STEER_HYBRID)",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    mode = DriveMode[
        node.get_parameter("drive_mode").get_parameter_value().string_value
    ]

    print(
        """Press w, a, s, d, q, e accordingly to move; space for stopping and Esckey for exiting.
        Be careful not to hold these keys, as these keys control the acceleration of the bot,
        and u dont want the bot flying out!!"""
    )
    print("Current Drive Mode: " + str(mode).split(".")[-1])
    enabled = True
    turn_state = 0
    while enabled:
        char = getch()
        if ord(char) == 27:
            print("Exiting")
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0
            enabled = False
        elif char == "w":
            print("Accelerated")
            vel_msg.linear.x += 0.008
        elif char == "s":
            print("Accelerated the opposite direction")
            vel_msg.linear.x -= 0.008
        elif char == "d":
            print("Turning right")
            vel_msg.angular.z -= 0.008
        elif char == "a":
            print("Turning left")
            vel_msg.angular.z += 0.008
        elif char == "q":
            print("Strafing left")
            if mode == DriveMode.SWERVE_DRIVE:
                vel_msg.linear.y += 0.008
            elif mode == DriveMode.TANK_STEER_HYBRID:
                turn_state = (1 if turn_state >= 0 else 0)
                pulse_msg.front_left_pulse = (75.0 if turn_state == 1 else 0.0)
                pulse_msg.front_right_pulse = (75.0 if turn_state == 1 else 0.0)
        elif char == "e":
            print("Strafing right")
            if mode == DriveMode.SWERVE_DRIVE:
                vel_msg.linear.y -= 0.008
            elif mode == DriveMode.TANK_STEER_HYBRID:
                turn_state = (-1 if turn_state <= 0 else 0)
                pulse_msg.front_left_pulse = (-75.0 if turn_state == -1 else 0.0)
                pulse_msg.front_right_pulse = (-75.0 if turn_state == -1 else 0.0)
        elif ord(char) == 32:
            print("Stopping")
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0
            pulse_msg.front_left_pulse = 0.0
            pulse_msg.front_right_pulse = 0.0
            turn_state = 0

        str_msg = f"lin x: {vel_msg.linear.x}, lin y: {vel_msg.linear.y}, ang z: {vel_msg.angular.z}"
        if mode == DriveMode.TANK_STEER_HYBRID:
            str_msg += f", FL swivel pulses: {pulse_msg.front_left_pulse}, FR swivel pulses: {pulse_msg.front_right_pulse}"
        print(str_msg)
        vel_msg.linear.z = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.x = 0.0
        publisher.publish(vel_msg)
        pulse_publisher.publish(pulse_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()