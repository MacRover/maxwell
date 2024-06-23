#!/usr/bin/env python3
"""
Main CAN Transmission Queue
"""
from typing import List
import threading
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from can import Message, ThreadSafeBus, Notifier

from custom_interfaces.msg import CANraw

class CANQueue(Node):
    def __init__(self):
        super().__init__("queue")
        self.declare_parameter(
            "queue_size",
            10,
            ParameterDescriptor(
                description="CAN bitrate. Passed to python-can",
                type=ParameterType.PARAMETER_INTEGER,
            ),
        )
        self.buffer_len = self.get_parameter("queue_size").get_parameter_value().integer_value
        self.q: List[CANraw] = list()
        self.pub = self.create_publisher(CANraw, "/can/can_out", 10)
        self.sub = self.create_subscription(
            CANraw, "/can/queue",
            (lambda msg: self.q.append(msg)), 10
        )
    
    def TX(self):
        if len(self.q) == 0:
            return
        if len(self.q) > self.buffer_len:
            raise Exception("CAN Queue Full")
        msg: CANraw = self.q.pop(0)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    queue = CANQueue()
    spin_thread = threading.Thread(target=rclpy.spin, args=(queue,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            queue.TX()
    except (KeyboardInterrupt):
        pass

    rclpy.try_shutdown()
    spin_thread.join()

if __name__ == "__main__":
    main()
    