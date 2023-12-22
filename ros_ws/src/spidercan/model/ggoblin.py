#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from can import Message
from can.interface import Bus

from custom_interfaces.msg import CANraw


class GGoblin(Node):
    def __init__(self):
        super().__init__("ggoblin")
        self.subscription = self.create_subscription(
            CANraw, "/can/can_out", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%d"' % msg.data)  # CHANGE
        print(msg)
        bus = Bus(interface="socketcan", channel="can0", bitrate=500000)
        message = Message(
            is_extended_id=msg.extended, arbitration_id=msg.address, data=msg.data
        )
        bus.send(message)


def main(args=None):
    rclpy.init(args=args)

    ggoblin = GGoblin()
    rclpy.spin(ggoblin)

    ggoblin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
