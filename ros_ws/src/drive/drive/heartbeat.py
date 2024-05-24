#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HeartBeatNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_repeater")
        self.hz = 50
        self.vel_msg = Twist()

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(
            Twist, "/vel_state", 
            self._vel_callback,
            10
        )
        self.timer = self.create_timer(
            1 / self.hz,
            (lambda: self.pub.publish(self.vel_msg))
        )
    
    def _vel_callback(self, msg):
        self.vel_msg = msg


def main():
    rclpy.init(args=None)
    try:
        heartbeat = HeartBeatNode()
        rclpy.spin(heartbeat)
    except KeyboardInterrupt:
        pass
    
    heartbeat.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
