#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from can import Message, ThreadSafeBus

from custom_interfaces.msg import CANraw


class Writer(Node):
    def __init__(self, bus, topic):
        super().__init__("writer")
        self.bus = bus
        self.topic = topic
        self.subscription = self.create_subscription(
            CANraw, self.topic, self.listener_callback, 10
        )

    def listener_callback(self, msg):
        message = Message(
            is_extended_id=msg.extended, arbitration_id=msg.address, data=msg.data
        )
        self.bus.send(message)
        self.get_logger().info(f"CAN TX addr:{msg.address}, data:{msg.data}")


def main(args=None):
    rclpy.init(args=args)

    bus = ThreadSafeBus(interface="socketcan", channel="can0", bitrate=500000)
    writer = Writer(bus=bus, topic="/can/can_out")
    rclpy.spin(writer)

    bus.shutdown()
    writer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
