#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

DEAD_ZONE = 0.05
TOP_SPEED = 1.5

class XboxDrive(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        self.vel_msg = Twist()
        self.pub = self.create_publisher(Twist, "/vel_state", 10)
        self.sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10)
    
    def _joy_callback(self, msg):
        self.vel_msg.linear.x = self._scale_with_deadzone(msg.axes[1])
        self.vel_msg.linear.y = self._scale_with_deadzone(msg.axes[0])
        self.vel_msg.angular.z = self._scale_with_deadzone(msg.axes[3])
        self.pub.publish(vel_msg)

    def _scale_with_deadzone(self, axes):
        if abs(axes) < DEAD_ZONE:
            return 0.0
        return self._map(axes, -1.0, 1.0, -TOP_SPEED, TOP_SPEED)

    def _map(self, value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def main():
    rclpy.init(args=None)
    try:
        xbox = XboxDrive()
        rclpy.spin(xbox)
    except KeyboardInterrupt:
        pass
    
    xbox.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
