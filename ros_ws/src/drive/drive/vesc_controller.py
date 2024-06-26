#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import *
from .VESC import *


class VescController(Node):
    def __init__(self):
        super().__init__("vesc_controller")
        self.sub = self.create_subscription(
            SwerveModulesList, 
            "/modules_command",
            self._callback, 10)
        
        self.pub = self.create_publisher(CANraw, "/can/can_out", 10)

        self.vfl = VESC(VESC_ID.FRONT_LEFT)
        self.vfr = VESC(VESC_ID.FRONT_RIGHT)
        self.vbl = VESC(VESC_ID.BACK_LEFT)
        self.vbr = VESC(VESC_ID.BACK_RIGHT)
    
    def _callback(self, msg):
        self.vfl.set_speed_mps(msg.front_left.speed)
        self.vfr.set_speed_mps(msg.front_right.speed)
        self.vbl.set_speed_mps(msg.rear_left.speed)
        self.vbr.set_speed_mps(msg.rear_right.speed)

        self.pub.publish(self.vfl.get_can_message())
        self.pub.publish(self.vfr.get_can_message())
        self.pub.publish(self.vbl.get_can_message())
        self.pub.publish(self.vbr.get_can_message())


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
