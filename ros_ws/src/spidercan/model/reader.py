#!/usr/bin/env python3
"""Read data from a CAN device and publish as a CANraw topic.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from can import Message, ThreadSafeBus, Notifier

from custom_interfaces.msg import CANraw


class Reader(Node):
    """Reads data from CAN bus device and publishes as a topic

    Args:
        topic (str): topic to publish CAN data to. Default: "/can/can_in"
        interface (str): type of CAN device to interface with. Passed to python-can. Default: "socketcan"
        channel (str): name of CAN device to interface with. Passed to python-can. Default: "can0"
        bitrate (int): transmission bitrate for CAN connection. Passed to python-can. Default: 500000
    """

    def __init__(self):
        super().__init__("reader")
        self.declare_parameter(
            "topic",
            "/can/can_in",
            ParameterDescriptor(
                description="Name of topic to publish to",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "interface",
            "socketcan",
            ParameterDescriptor(
                description="CAN interface. Passed to python-can",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "channel",
            "can0",
            ParameterDescriptor(
                description="CAN device. Passed to python-can",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "bitrate",
            500000,
            ParameterDescriptor(
                description="CAN bitrate. Passed to python-can",
                type=ParameterType.PARAMETER_INTEGER,
            ),
        )

        self.publisher = self.create_publisher(
            CANraw, self.get_parameter("topic").get_parameter_value().string_value, 10
        )
        self.bus = ThreadSafeBus(
            interface=self.get_parameter("interface")
            .get_parameter_value()
            .string_value,
            channel=self.get_parameter("channel").get_parameter_value().string_value,
            bitrate=self.get_parameter("bitrate").get_parameter_value().integer_value,
        )
        self.notifier = Notifier(self.bus, listeners=[self.can_recv_callback])
        self.notifier.add_listener(self.can_recv_callback)
        self.bus.recv()

    def can_recv_callback(self, msg: Message):
        ros_msg = CANraw(
            address=msg.arbitration_id, data=msg.data, extended=msg.is_extended_id
        )
        self.publisher.publish(ros_msg)
        self.get_logger().debug(f"CAN RX addr:{ros_msg.address}, data:{ros_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    reader = Reader()
    rclpy.spin(reader)


if __name__ == "__main__":
    main()
