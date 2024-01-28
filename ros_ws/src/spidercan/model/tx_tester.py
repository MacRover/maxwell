#!/usr/bin/env python3
import struct
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import CANraw


class TxTester(Node):
    _tx_data: int

    def __init__(self, topic: str):
        super().__init__("txtester")
        self.publisher = self.create_publisher(CANraw, topic, 10)
        
        self.timer = self.create_timer(0.001, self.timer_callback)
        self._tx_data = 0
    
    def timer_callback(self):
        msg = CANraw()
        msg.address = 1
        msg.data = struct.pack('Q',self._tx_data)
        self.publisher.publish(msg=msg)
        self._tx_data += 1


def main(args=None):
    rclpy.init(args=args)

    tx_tester = TxTester(topic="/can/can_out")
    rclpy.spin(tx_tester)

    tx_tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
