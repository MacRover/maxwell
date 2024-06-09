#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

DEAD_ZONE = 0.01
TOP_SPEED = 0.5
TURBO_FACTOR = 3

class XboxDrive(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        self.vel_msg = Twist()
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10)
    
    def _joy_callback(self, msg):
        speed = TOP_SPEED
        if (msg.buttons[5]):
            speed = speed * TURBO_FACTOR
            
        self.vel_msg.linear.x = self._scale_with_deadzone(msg.axes[1],speed)
        self.vel_msg.linear.y = self._scale_with_deadzone(msg.axes[0],speed)
        self.vel_msg.angular.z = self._scale_with_deadzone(msg.axes[3],speed)
        self.pub.publish(self.vel_msg)

    def _scale_with_deadzone(self, axes, speed):
        if abs(axes) < DEAD_ZONE:
            return 0.0
        return self._map(axes, -1.0, 1.0, -speed, speed)

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
