#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
class LoRaNode(Node):
    def __init__(self):
        super().__init__('lora_node')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.sub = self.create_subscription(
            String, 'rover/info', self.cb, 10)

    def cb(self, msg):
        self.ser.write((msg.data + '\n').encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = LoRaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
