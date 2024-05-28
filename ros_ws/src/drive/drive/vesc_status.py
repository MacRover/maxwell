#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from custom_interfaces.msg import CANraw
from .VESC import *

class VescStatus(Node):
    def __init__(self):
        super().__init__("vesc_status_node")
        self.sub_status = self.create_subscription(
            CANraw,
            "/can/can_in",
            self._callback, 10
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
        self.status: Status = Status[
            self.get_parameter("status").get_parameter_value().string_value
        ]
        self.motor: VESC_ID = VESC_ID[
            self.get_parameter("motor").get_parameter_value().string_value
        ]

    def _callback(self, msg):
        if ((msg.address & 0xff) != self.motor.value and 
            (msg.address >> 8) != self.status.value):
            return
            
        if (self.status == Status.STATUS_1):
            rpm = (msg.data >> 32)
            cur = ((msg.data >> 16) & 0xffff) / 10.0
            dc = (msg.data & 0xffff) / 1000.0
            print(f"RPM:{rpm}, Current:{cur}A, Duty Cycle:{dc}%")

        elif (self.status == Status.STATUS_2):
            ampH = (msg.data >> 32) / 10000.0
            ampHC = (msg.data & 0xffffffff) / 10000.0
            print(f"AmpH:{ampH}Ah, AmpHChg:{ampHC}Ah")

        elif (self.status == Status.STATUS_3):
            wH = (msg.data >> 32) / 10000.0
            wHC = (msg.data & 0xffffffff) / 10000.0
            print(f"WattH:{wH}Ah, WattHChg:{wHC}Ah")

        elif (self.status == Status.STATUS_4):
            tFET = (msg.data >> 48) / 10.0
            tMotor = ((msg.data >> 32) & 0xffff) / 10.0
            curIn = ((msg.data >> 16) & 0xffff) / 10.0
            pos = (msg.data & 0xffff) / 50.0
            print(f"tFET:{tFET}DegC, tMotor:{tMotor}DegC, curIn:{curIn}A, Pos:{pos}Deg")

        elif (self.status == Status.STATUS_5):
            tach = (msg.data >> 32) / 6.0
            vIn = ((msg.data >> 16) & 0xffff) / 10.0
            print(f"tach:{tach}EREV, VoltsIn:{vIn}V")
            
        elif (self.status == Status.STATUS_6):
            adc1 = (msg.data >> 48) / 1000.0
            adc2 = ((msg.data >> 32) & 0xffff) / 1000.0
            adc3 = ((msg.data >> 16) & 0xffff) / 1000.0
            ppm = (msg.data & 0xffff) / 1000.0
            print(f"ADC1:{adc1}V, ADC2:{adc2}V, ADC3:{adc3}V, PPM:{ppm}%")


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