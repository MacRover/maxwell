#! /usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from custom_interfaces.msg import *
from .VESC import *

THRES = 15.0

class VescController(Node):
    def __init__(self):
        super().__init__("vesc_controller")
        self.sub = self.create_subscription(
            SwerveModulesList, 
            "/drive/modules_command",
            self._callback, 10)
        self.sub_odom = self.create_subscription(
            SwerveModulesList, 
            "/drive/drive_modules",
            self._odom_callback, 10)

        self.declare_parameter(
            "can_rate",
            10,
            ParameterDescriptor(
                description="Rate of CAN frame",
                type=ParameterType.PARAMETER_INTEGER,
            ),
        )
        self.declare_parameter(
            "wait_until_positioned",
            True,
            ParameterDescriptor(
                description="Drive motors only once swerve is in position",
                type=ParameterType.PARAMETER_BOOL,
            ),
        )
        
        self.wait_until_pos = self.get_parameter("wait_until_positioned").get_parameter_value().bool_value

        self.fr_theta = 0.0
        self.fl_theta = 0.0
        self.bl_theta = 0.0
        self.br_theta = 0.0

        self.delay_sec = 1/(4*self.get_parameter("can_rate").get_parameter_value().integer_value)
        
        self.pub = self.create_publisher(CANstamped, "/can/can_out_queue", 10)

        self.vfl = VESC(VESC_ID.FRONT_LEFT)
        self.vfr = VESC(VESC_ID.FRONT_RIGHT)
        self.vbl = VESC(VESC_ID.BACK_LEFT)
        self.vbr = VESC(VESC_ID.BACK_RIGHT)
        self.vbl.set_gear_reduction(4.67)
        self.vbr.set_gear_reduction(4.67)
        self.vfl.set_gear_reduction(4.67)
        self.vfr.set_gear_reduction(4.67)
    
    def _callback(self, msg):
        if any([abs(self.fr_theta - msg.front_right.angle) < THRES,
                abs(self.fl_theta - msg.front_left.angle) < THRES,
                abs(self.br_theta - msg.rear_right.angle) < THRES,
                abs(self.bl_theta - msg.rear_left.angle) < THRES]) or not self.wait_until_pos:

            self.vfl.set_speed_mps(msg.front_left.speed)
            self.vfr.set_speed_mps(msg.front_right.speed)
            self.vbl.set_speed_mps(msg.rear_left.speed)
            self.vbr.set_speed_mps(msg.rear_right.speed)
        else:
            self.vfl.set_speed_mps(0.0)
            self.vfr.set_speed_mps(0.0)
            self.vbl.set_speed_mps(0.0)
            self.vbr.set_speed_mps(0.0)
        
        time = self.get_clock().now()
        future_stamp = time + Duration(seconds=self.delay_sec * 0.5)
        self.pub.publish(CANstamped(stamp=future_stamp.to_msg(), can_raw=self.vfl.get_can_message()))
        future_stamp = time + Duration(seconds=self.delay_sec * 1.5)
        self.pub.publish(CANstamped(stamp=future_stamp.to_msg(), can_raw=self.vfr.get_can_message()))
        future_stamp = time + Duration(seconds=self.delay_sec * 2.5)
        self.pub.publish(CANstamped(stamp=future_stamp.to_msg(), can_raw=self.vbl.get_can_message()))
        future_stamp = time + Duration(seconds=self.delay_sec * 3.5)
        self.pub.publish(CANstamped(stamp=future_stamp.to_msg(), can_raw=self.vbr.get_can_message()))
    
    def _odom_callback(self, msg):
        self.fr_theta = msg.front_right.angle
        self.fl_theta = msg.front_left.angle
        self.br_theta = msg.rear_right.angle
        self.bl_theta = msg.rear_left.angle

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = VescController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    controller.destroy_node()


if __name__ == "__main__":
    main()
