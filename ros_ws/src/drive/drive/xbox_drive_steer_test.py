#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64

class XboxDriveTest(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        self.pub_fl = self.create_publisher(Int64, "/test/front_left/rad_speed", 10)
        self.pub_fr = self.create_publisher(Int64, "/test/front_right/rad_speed", 10)
        self.sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10)
    
    def _joy_callback(self, msg):
        rad_speed_msg_1 = Int64()
        rad_speed_msg_2 = Int64()

        if (msg.axes[3] > 0.9):
            if (msg.buttons[4]):
                rad_speed_msg_1.data = 255
            elif (msg.buttons[5]):
                rad_speed_msg_2.data = 255
            else:
                rad_speed_msg_1.data = 255
                rad_speed_msg_2.data = 255
        elif (msg.axes[3] < -0.9):
            if (msg.buttons[4]):
                rad_speed_msg_1.data = -255
            elif (msg.buttons[5]):
                rad_speed_msg_2.data = -255
            else:
                rad_speed_msg_1.data = -255
                rad_speed_msg_2.data = -255

        self.pub_fl.publish(rad_speed_msg_1)
        self.pub_fr.publish(rad_speed_msg_2)


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
