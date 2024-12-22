#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Twist

from custom_interfaces.msg import SwerveModulePulse

from .drive_controller import DriveMode

class HeartBeatNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_repeater")
        self.declare_parameter(
            "topic_rate",
            10,
            ParameterDescriptor(
                description="Rate of topics published (in hz)",
                type=ParameterType.PARAMETER_INTEGER,
            ),
        )
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
        self.hz = self.get_parameter("topic_rate").get_parameter_value().integer_value
        self.vel_msg = Twist()
        self.pulse_msg = SwerveModulePulse()

        self.pub_vel = self.create_publisher(Twist, "/cmd_vel_repeat", 10)
        self.sub_vel = self.create_subscription(
            Twist, "/cmd_vel", 
            self._vel_callback,
            10
        )
        self.timer_vel = self.create_timer(
            1 / self.hz,
            (lambda: self.pub_vel.publish(self.vel_msg))
        )

        if (self.mode == DriveMode.TANK_STEER_HYBRID):
            self.sub_pulses = self.create_subscription(
                SwerveModulePulse, "/rad_pulses",
                self._pulse_callback,
                10
            )
            self.pub_pulse = self.create_publisher(SwerveModulePulse, "/rad_pulses_repeat", 10)
            self.timer_pulse = self.create_timer(
                1 / self.hz,
                (lambda: self.pub_pulse.publish(self.pulse_msg))
            )
    
    def _vel_callback(self, msg: Twist):
        self.vel_msg = msg

    def _pulse_callback(self, msg: SwerveModulePulse):
        self.pulse_msg = msg

def main():
    rclpy.init(args=None)
    try:
        heartbeat = HeartBeatNode()
        rclpy.spin(heartbeat)
    except KeyboardInterrupt:
        pass
    
    heartbeat.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
