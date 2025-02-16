#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from custom_interfaces.msg import SwerveModulePulse

from .drive_controller import DriveMode

DEAD_ZONE = 0.05
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

        x = msg.axes[1]
        y = msg.axes[0]
            
        self.vel_msg.linear.x = self._scale_with_deadzone(x,speed)
        self.vel_msg.linear.y = self._scale_with_deadzone(y,speed)
        self.vel_msg.angular.z = 0.0
        if (msg.buttons[5]):
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.angular.z = 0.001+self._scale_with_deadzone(msg.axes[3],speed)
        self.pub_cmd_vel.publish(self.vel_msg)


class XboxTankSteerDrive(XboxDrive):
    def __init__(self, pub_cmd_vel: Publisher, pub_rad_pulses: Publisher):
        self.pub_cmd_vel = pub_cmd_vel
        self.pub_rad_pulses = pub_rad_pulses
        self.vel_msg = Twist()
        self.STEPS = 75.0
    
    def joy_callback(self, msg: Joy) -> None:
        rad_pulse_msg  = SwerveModulePulse()

        signnum = (lambda n: (n > 0) - (n < 0))

        if (abs(msg.axes[3]) > 0.4):
            rad_pulse_msg.front_left_pulse = self.STEPS*signnum(msg.axes[3])*-1
            rad_pulse_msg.front_right_pulse = self.STEPS*signnum(msg.axes[3])*-1
            if (msg.buttons[4]):
                rad_pulse_msg.front_right_pulse = 0.0
                rad_pulse_msg.rear_right_pulse = self.STEPS*signnum(msg.axes[3])*-1
            if (msg.buttons[5]):
                rad_pulse_msg.front_left_pulse = 0.0
                rad_pulse_msg.rear_left_pulse = self.STEPS*signnum(msg.axes[3])*-1
        else:
            rad_pulse_msg.front_left_pulse  = 0.0
            rad_pulse_msg.front_right_pulse = 0.0
            rad_pulse_msg.rear_right_pulse  = 0.0
            rad_pulse_msg.rear_left_pulse   = 0.0

        self.pub_rad_pulses.publish(rad_pulse_msg)

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
        
        self.pub_cmd_vel = self.create_publisher(Twist, "/drive/cmd_vel", 10)

        if (self.mode == DriveMode.SWERVE_DRIVE):
            self.xbox_drive = XboxSwerveDrive(self.pub_cmd_vel)
        elif (self.mode == DriveMode.TANK_STEER_HYBRID):
            self.pub = self.create_publisher(SwerveModulePulse, "/drive/rad_pulses", 10)
            self.xbox_drive = XboxTankSteerDrive(self.pub_cmd_vel, self.pub)

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
