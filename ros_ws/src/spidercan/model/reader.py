#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from can import (
    Message,
    ThreadSafeBus,
    Notifier,
    BufferedReader,
    Listener,
    BusABC,
)
from custom_interfaces.msg import CANraw


class Reader(Node):
    def __init__(self, bus: BusABC, notifier: Notifier, topic: str):
        super().__init__("reader")
        self.publisher = self.create_publisher(CANraw, topic, 10)
        self.bus = bus
        self.notifier = notifier
        self.notifier.add_listener(self.can_recv_callback)

        self.bus.recv()

    def can_recv_callback(self, msg: Message):
        ros_msg = CANraw(
            address=msg.arbitration_id, data=msg.data, extended=msg.is_extended_id
        )
        self.publisher.publish(ros_msg)
        self.get_logger().info(f"CAN RX addr:{ros_msg.address}, data:{ros_msg.data}")


def main(args=None):
    rclpy.init(args=args)

    bus_instance = ThreadSafeBus(interface="socketcan", channel="can0", bitrate=500000)
    notifier = Notifier(bus=bus_instance, listeners=[])
    reader = Reader(bus=bus_instance, notifier=notifier, topic="/can/can_in")
    rclpy.spin(reader)

    notifier.stop()
    bus_instance.shutdown()
    reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
