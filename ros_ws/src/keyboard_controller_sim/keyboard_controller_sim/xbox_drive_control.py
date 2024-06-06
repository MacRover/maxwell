#!/usr/bin/env python3
from math import atan2, sqrt, pi
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

MAX_STICK_ANGLE=pi/2+0.2
MAX_SPEED=1.4


class XboxController(Node):
    def __init__(self, name):
        super().__init__(name)
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
        self.rotating = False
        self.create_subscription(Joy, "/joy", self.drive_joysticks, 10)
    
    def drive_joysticks(self, msg):
        x_t, y_t = msg.axes[0], msg.axes[1]
        x_r = msg.axes[3]

        if (msg.buttons[0]):
            self.rotating = True
        if (msg.buttons[1]): 
            self.rotating = False

        theta = atan2(x_t, y_t)
        speed = sqrt(x_t ** 2 + y_t ** 2) * MAX_SPEED
        
        if (theta > MAX_STICK_ANGLE):
            theta -= pi
            speed = -speed
        elif (theta < -MAX_STICK_ANGLE):
            theta += pi
            speed = -speed
        
        if (self.rotating):
            speed = x_r * MAX_SPEED
            self.swerve_msg.data = [-pi/4, pi/4, pi/4, -pi/4]
            self.wheel_msg.data = [-speed, speed, -speed, speed]
        else:
            self.swerve_msg.data = [theta, theta, theta, theta]
            self.wheel_msg.data = [speed, speed, speed, speed]
            
        self.publisher_swerve_.publish(self.swerve_msg)
        self.publisher_wheel_.publish(self.wheel_msg)


def main(args=None):
    rclpy.init(args=args)
    xbox_node = XboxController("drive_controller")

    try:
        rclpy.spin(xbox_node)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
