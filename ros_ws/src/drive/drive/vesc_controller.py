#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum

from custom_interfaces.msg import *
from .VESC import VESC

class VESC_ID(Enum):
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3


class VescController(Node):
    def __init__(self):
        super().__init__("vesc_controller")
        self.sub = self.create_subscription(
            SwerveModulesList, 
            "/modules_command",
            self._callback, 10)
        
        self.pub = self.create_publisher(CANraw, "/can/can_out", 10)

        self.front_left = VESC(VESC_ID.FRONT_LEFT)
        self.front_right = VESC(VESC_ID.FRONT_RIGHT)
        self.back_left = VESC(VESC_ID.BACK_LEFT)
        self.back_right = VESC(VESC_ID.BACK_RIGHT)
    
    def _callback(self, msg):
        self.front_left.set_speed_mps(msg[0].speed)
        self.front_right.set_speed_mps(msg[1].speed)
        self.back_left.set_speed_mps(msg[2].speed)
        self.back_right.set_speed_mps(msg[3].speed)

        self.pub.publish(front_left.get_can_message())
        self.pub.publish(front_right.get_can_message())
        self.pub.publish(back_left.get_can_message())
        self.pub.publish(back_right.get_can_message())


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
