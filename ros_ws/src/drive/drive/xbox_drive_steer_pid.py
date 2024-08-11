#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class XboxDriveTestPID(Node):
    def __init__(self):
        super().__init__("xbox_pid_controller")
        self.pub = self.create_publisher(Float32, "/test/rad_pos", 10)
        self.sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10)
    
    def _joy_callback(self, msg):
        position = Float32()
        position.data = self._map(msg.axes[3], -1.0, 1.0, -90.0, 90.0)
        self.pub.publish(position)

    def _map(self, value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

def main():
    rclpy.init(args=None)
    try:
        xbox = XboxDriveTestPID()
        rclpy.spin(xbox)
    except KeyboardInterrupt:
        pass
    
    xbox.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()