#!/usr/bin/env python3
from math import pi

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from custom_interfaces.msg import *


class DriveModulesOdom(Node):
    def __init__(self):
        super().__init__("drive_modules_odom")

        self.declare_parameter(
            "odom_rate",
            20,
            ParameterDescriptor(
                description="Rate of Wheel Odometry",
                type=ParameterType.PARAMETER_INTEGER,
            ),
        )

        # ----------- fr,  fl,  br,  bl
        self.theta = [0.0, 0.0, 0.0, 0.0]
        self.mps   = [0.0, 0.0, 0.0, 0.0]
        self.wheel_diameter = 0.22

        self.drive_modules_msg = SwerveModulesList()

        self.sub_fr_status = self.create_subscription(
            RadStatus, 
            "/front_right/rad_status",
            self._fr_rad_callback, 10)
        self.sub_fl_status = self.create_subscription(
            RadStatus, 
            "/front_left/rad_status",
            self._fl_rad_callback, 10)
        self.sub_br_status = self.create_subscription(
            RadStatus, 
            "/rear_right/rad_status",
            self._br_rad_callback, 10)
        self.sub_bl_status = self.create_subscription(
            RadStatus, 
            "/rear_left/rad_status",
            self._bl_rad_callback, 10)
        
        self.sub_fr_vstatus = self.create_subscription(
            Int32, 
            "/front_right/vesc_rpm",
            self._fr_vesc_callback, 10)
        self.sub_fl_vstatus = self.create_subscription(
            Int32, 
            "/front_left/vesc_rpm",
            self._fl_vesc_callback, 10)
        self.sub_br_vstatus = self.create_subscription(
            Int32, 
            "/rear_right/vesc_rpm",
            self._br_vesc_callback, 10)
        self.sub_bl_vstatus = self.create_subscription(
            Int32, 
            "/rear_left/vesc_rpm",
            self._bl_vesc_callback, 10)
        self.odom_pub = self.create_publisher(SwerveModulesList, "/drive_modules", 10)
        self.timer = self.create_timer(
            1.0 / (self.get_parameter("odom_rate").get_parameter_value().integer_value),
            self._timer_callback
        )
    
    def _fr_rad_callback(self, msg):
        self.theta[0] = msg.angle - 120.0
    def _fl_rad_callback(self, msg):
        self.theta[1] = msg.angle - 120.0
    def _br_rad_callback(self, msg):
        self.theta[2] = msg.angle - 120.0
    def _bl_rad_callback(self, msg):
        self.theta[3] = msg.angle - 120.0
    
    def _fr_vesc_callback(self, msg):
        self.mps[0] = msg.data * (pi * self.wheel_diameter) / 60.0
    def _fl_vesc_callback(self, msg):
        self.mps[1] = msg.data * (pi * self.wheel_diameter) / 60.0
    def _br_vesc_callback(self, msg):
        self.mps[2] = msg.data * (pi * self.wheel_diameter) / 60.0
    def _bl_vesc_callback(self, msg):
        self.mps[3] = msg.data * (pi * self.wheel_diameter) / 60.0
    
    def _timer_callback(self):
        self.drive_modules_msg.front_right.speed = self.mps[0]
        self.drive_modules_msg.front_left.speed = self.mps[1]
        self.drive_modules_msg.rear_right.speed = self.mps[2]
        self.drive_modules_msg.rear_left.speed = self.mps[3]

        self.drive_modules_msg.front_right.angle = self.theta[0]
        self.drive_modules_msg.front_left.angle = self.theta[1]
        self.drive_modules_msg.rear_right.angle = self.theta[2]
        self.drive_modules_msg.rear_left.angle = self.theta[3]

        self.odom_pub.publish(self.drive_modules_msg)


def main():
    rclpy.init(args=None)
    try:
        drive_module_odom = DriveModulesOdom()
        rclpy.spin(drive_module_odom)
    except KeyboardInterrupt:
        drive_module_odom.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

