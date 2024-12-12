#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from .drive_controller import DriveMode

DEAD_ZONE = 0.01
TOP_SPEED = 0.5
TURBO_FACTOR = 3


class XboxDrive:
    # Joystick callback function
    def joy_callback(self, msg: Joy) -> None:
        pass

    def _scale_with_deadzone(self, axes, speed):
        if abs(axes) < DEAD_ZONE:
            return 0.0
        return self._map(axes, -1.0, 1.0, -speed, speed)

    def _map(self, value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


class XboxSwerveDrive(XboxDrive):
    def __init__(self, pub_cmd_vel: Publisher):
        self.pub_cmd_vel = pub_cmd_vel
        self.vel_msg = Twist()
    
    def joy_callback(self, msg: Joy) -> None:
        speed = TOP_SPEED*self._map(msg.axes[5], 1.0, -1.0, 1.0, TURBO_FACTOR)
            
        self.vel_msg.linear.x = self._scale_with_deadzone(msg.axes[1],speed)
        self.vel_msg.linear.y = self._scale_with_deadzone(msg.axes[0],speed)
        self.vel_msg.angular.z = 0.0
        if (msg.buttons[5]):
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.angular.z = 0.001+self._scale_with_deadzone(msg.axes[3],speed)
        self.pub_cmd_vel.publish(self.vel_msg)


class XboxTankSteerDrive(XboxDrive):
    def __init__(self, pub_cmd_vel: Publisher, pub_rad_fr: Publisher, pub_rad_fl: Publisher):
        self.pub_cmd_vel = pub_cmd_vel
        self.pub_rad_fr = pub_rad_fr
        self.pub_rad_fl = pub_rad_fl
        self.vel_msg = Twist()
    
    def joy_callback(self, msg: Joy) -> None:
        rad_speed_msg_1 = Int64()
        rad_speed_msg_2 = Int64()

        if (msg.axes[3] > 0.5):
            rad_speed_msg_1 = (0 if msg.buttons[4] else 255)
            rad_speed_msg_2 = (0 if msg.buttons[5] else 255)
        elif (msg.axes[3] < -0.5):
            rad_speed_msg_1 = (0 if msg.buttons[4] else -255)
            rad_speed_msg_2 = (0 if msg.buttons[5] else -255)

        self.pub_rad_fl.publish(rad_speed_msg_1)
        self.pub_rad_fr.publish(rad_speed_msg_2)

        speed = TOP_SPEED*self._map(msg.axes[5], 1.0, -1.0, 1.0, TURBO_FACTOR)   
        self.vel_msg.linear.x = self._scale_with_deadzone(msg.axes[1],speed)
        self.pub_cmd_vel.publish(self.vel_msg)


class XboxDriveController(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        self.declare_parameter(
            "drive_mode",
            "SWERVE_DRIVE",
            ParameterDescriptor(
                description="Method of Driving (SWERVE_DRIVE | TANK_STEER_HYBRID)",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.mode: DriveMode = DriveMode[
            self.get_parameter("drive_mode").get_parameter_value().string_value
        ]
        
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        if (self.mode == DriveMode.SWERVE_DRIVE):
            self.xbox_drive = XboxSwerveDrive(self.pub_cmd_vel)
        elif (self.mode == DriveMode.TANK_STEER_HYBRID):
            self.pub_fl = self.create_publisher(Int64, "/front_left/rad_speed", 10)
            self.pub_fr = self.create_publisher(Int64, "/front_right/rad_speed", 10)
            self.xbox_drive = XboxTankSteerDrive(self.pub_cmd_vel, self.pub_fr, self.pub_fl)

        self.sub = self.create_subscription(
            Joy, "/joy", self.xbox_drive.joy_callback, 10)


def main():
    rclpy.init(args=None)
    try:
        xbox = XboxDriveController()
        rclpy.spin(xbox)
    except KeyboardInterrupt:
        pass
    
    xbox.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
