#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import *
from .VESC import VESC

class VescController(Node):
    def __init__(self):
        super().__init__("vesc_controller")
        self.sub = self.create_subscription(
            SwerveModulesList, 
            "/modules_command",
            self._callback, 10)
        
        self.pub = self.create_publisher(CANraw, "/can/can_out", 10)
    
    def _callback(self, msg):
        pass


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
