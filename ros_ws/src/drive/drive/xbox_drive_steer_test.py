#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64

class XboxDriveTest(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        self.pub = self.create_publisher(Int64, "/test/rad_speed", 10)
        self.sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10)
    
    def _joy_callback(self, msg):
        rad_speed_msg = Int64()
        if msg.buttons[4]:
            rad_speed_msg.data = 100
        elif msg.buttons[5]:
            rad_speed_msg.data = -100
        else:
            rad_speed_msg.data = 0
        self.pub.publish(rad_speed_msg)


def main():
    rclpy.init(args=None)
    try:
        xbox = XboxDriveTest()
        rclpy.spin(xbox)
    except KeyboardInterrupt:
        pass
    
    xbox.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
