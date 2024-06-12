#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

import struct
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from custom_interfaces.msg import CANraw
from std_msgs.msg import Int32
from .VESC import *

MAX_CURRENT_DRAW = 65

class VescStatus(Node):
    def __init__(self):
        super().__init__("vesc_status_node")
        self.output: str = None
        self.sub_status = self.create_subscription(
            CANraw,
            "/can/can_in",
            self.status_callback, 10
        )
        self.declare_parameter(
            "status",
            "STATUS_1",
            ParameterDescriptor(
                description="CAN status command to listen to",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "motor",
            "FRONT_RIGHT",
            ParameterDescriptor(
                description="Motor status",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "status_rate",
            10,
            ParameterDescriptor(
                description="Publish rate of status messages",
                type=ParameterType.PARAMETER_INTEGER,
            ),
        )
        self.declare_parameter(
            "namespace",
            "front_right",
            ParameterDescriptor(
                description="Namespace of status topic",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "logging",
            False,
            ParameterDescriptor(
                description="Logging flag",
                type=ParameterType.PARAMETER_BOOL
            )
        )

        self.status: Status = Status[
            self.get_parameter("status").get_parameter_value().string_value
        ]
        self.motor: VESC_ID = VESC_ID[
            self.get_parameter("motor").get_parameter_value().string_value
        ]

        is_logging = self.get_parameter("logging").get_parameter_value().bool_value
        if is_logging:
            self.timer = self.create_timer(
                1 / (self.get_parameter("status_rate").get_parameter_value().integer_value),
                (lambda: self.get_logger().info(self.output) if self.output else None))

        self.pub_status = self.create_publisher(
            Int32, 
            "/"+self.get_parameter("namespace").get_parameter_value().string_value+"/vesc_rpm",
            10
        )
        self.rpm = Int32()
        self.max_cur = -1

    def status_callback(self, msg: CANraw):
        if ((msg.address & 0xff) != self.motor.value or 
            (msg.address >> 8) != self.status.value):
            return
        
        raw_data = struct.unpack(">q", msg.data)[0]
        if (self.status == Status.STATUS_1):
            rpm = (raw_data >> 32)
            cur = ((raw_data >> 16) & 0xffff) / 10.0
            dc = (raw_data & 0xffff) / 1000.0
            if (self.max_cur < cur and self.max_cur < MAX_CURRENT_DRAW):
                self.max_cur = cur
            self.output = f"RPM:{rpm}, Current:{cur}A, Max Current:{self.max_cur}A, Duty Cycle:{dc}%"
            self.rpm.data = rpm
            self.pub_status.publish(self.rpm)

        elif (self.status == Status.STATUS_2):
            ampH = (raw_data >> 32) / 10000.0
            ampHC = (raw_data & 0xffffffff) / 10000.0
            self.output = f"AmpH:{ampH}Ah, AmpHChg:{ampHC}Ah"

        elif (self.status == Status.STATUS_3):
            wH = (raw_data >> 32) / 10000.0
            wHC = (raw_data & 0xffffffff) / 10000.0
            self.output = f"WattH:{wH}Ah, WattHChg:{wHC}Ah"

        elif (self.status == Status.STATUS_4):
            tFET = (raw_data >> 48) / 10.0
            tMotor = ((raw_data >> 32) & 0xffff) / 10.0
            curIn = ((raw_data >> 16) & 0xffff) / 10.0
            pos = (raw_data & 0xffff) / 50.0
            self.output = \
            f"tFET:{tFET}DegC, tMotor:{tMotor}DegC, curIn:{curIn}A, Pos:{pos}Deg"

        elif (self.status == Status.STATUS_5):
            tach = (raw_data >> 32) / 6.0
            vIn = ((raw_data >> 16) & 0xffff) / 10.0
            self.output = f"tach:{tach}EREV, VoltsIn:{vIn}V"
            
        elif (self.status == Status.STATUS_6):
            adc1 = (raw_data >> 48) / 1000.0
            adc2 = ((raw_data >> 32) & 0xffff) / 1000.0
            adc3 = ((raw_data >> 16) & 0xffff) / 1000.0
            ppm = (raw_data & 0xffff) / 1000.0
            self.output = f"ADC1:{adc1}V, ADC2:{adc2}V, ADC3:{adc3}V, PPM:{ppm}%"


def main(args=None):
    rclpy.init(args=args)
    try:
        status = VescStatus()
        rclpy.spin(status)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    status.destroy_node()


if __name__ == "__main__":
    main()