#!/usr/bin/env python3
"""Subscribe to CANraw topic and publish to CAN bus
"""
import heapq
import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from can import Message, ThreadSafeBus

from custom_interfaces.msg import CANraw, CANstamped


class Writer(Node):
    """Writes data to CAN bus

    Args:
        raw_topic (str): topic to subscribe to for CAN data. Must be in CANraw message format. Default: "/can/can_out"
        interface (str): type of CAN device to interface with. Passed to python-can. Default: "socketcan"
        channel (str): name of CAN device to interface with. Passed to python-can. Default: "can0"
        bitrate (int): transmission bitrate for CAN connection. Passed to python-can. Default: 500000
    """

    def __init__(self):
        super().__init__("writer")

        self.p_queue: list[tuple[float, CANstamped]] = []

        self.declare_parameter(
            "raw_topic",
            "/can/can_out",
            ParameterDescriptor(
                description="Name of topic to subscribe to",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "queue_topic",
            "/can/can_out_queue",
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

        self.raw_subscription = self.create_subscription(
            CANraw,
            self.get_parameter("raw_topic").get_parameter_value().string_value,
            self.send_can_msg,
            15,
        )
        self.queue_subscription = self.create_subscription(
            CANstamped,
            self.get_parameter("queue_topic").get_parameter_value().string_value,
            self.enqueue_callback,
            15,
        )

        self.bus = ThreadSafeBus(
            interface=self.get_parameter("interface")
            .get_parameter_value()
            .string_value,
            channel=self.get_parameter("channel").get_parameter_value().string_value,
            bitrate=self.get_parameter("bitrate").get_parameter_value().integer_value,
        )

        self.thread = threading.Thread(target=self.can_main_receiver, daemon=True)
        self.thread.start()

    def send_can_msg(self, msg):
        message = Message(
            is_extended_id=msg.extended, arbitration_id=msg.address, data=msg.data
        )
        self.bus.send(message)
        self.get_logger().debug(f"CAN TX addr:{msg.address}, data:{msg.data}")
    
    def enqueue_callback(self, msg):
        time = msg.stamp.sec + msg.stamp.nanosec / 1e9
        heapq.heappush(self.p_queue, (time, msg))
    
    def can_main_receiver(self):
        while rclpy.ok():
            if not self.p_queue:
                continue
            _, next_msg = self.p_queue[0]
            if self.get_clock().now() >= Time.from_msg(next_msg.stamp):
                msg = heapq.heappop(self.p_queue)
                self.send_can_msg(msg[1].can_raw)
        return None


def main(args=None):
    rclpy.init(args=args)
    try:
        writer = Writer()
        rclpy.spin(writer)
    except KeyboardInterrupt:
        rclpy.try_shutdown()
        writer.bus.shutdown()
        writer.destroy_node()


if __name__ == "__main__":
    main()
