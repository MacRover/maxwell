#!/usr/bin/env python3
"""Subscribe to CANraw topic and publish to CAN bus
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from can import Message, ThreadSafeBus

from custom_interfaces.msg import CANraw


class Writer(Node):
    """Writes data to CAN bus

    Args:
        topic (str): topic to subscribe to for CAN data. Must be in CANraw message format. Default: "/can/can_out"
        interface (str): type of CAN device to interface with. Passed to python-can. Default: "socketcan"
        channel (str): name of CAN device to interface with. Passed to python-can. Default: "can0"
        bitrate (int): transmission bitrate for CAN connection. Passed to python-can. Default: 500000
    """

    def __init__(self):
        super().__init__("writer")

        # Set to true when testing
        self.VCAN_ENABLED = True

        self.declare_parameter(
            "topic",
            "/can/can_out",
            ParameterDescriptor(
                description="Name of topic to subscribe to",
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
            ("can0" if not self.VCAN_ENABLED else "vcan0"),
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

        self.subscription = self.create_subscription(
            CANraw,
            self.get_parameter("topic").get_parameter_value().string_value,
            self.new_message_callback,
            10,
        )

        self.bus = ThreadSafeBus(
            interface=self.get_parameter("interface")
            .get_parameter_value()
            .string_value,
            channel=self.get_parameter("channel").get_parameter_value().string_value,
            bitrate=self.get_parameter("bitrate").get_parameter_value().integer_value,
        )

    def new_message_callback(self, msg):
        message = Message(
            is_extended_id=msg.extended, arbitration_id=msg.address, data=msg.data
        )
        self.bus.send(message)
        self.get_logger().debug(f"CAN TX addr:{msg.address}, data:{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    writer = Writer()
    rclpy.spin(writer)


if __name__ == "__main__":
    main()
