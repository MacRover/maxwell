#! /usr/bin/env python3
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32



class anglePublisher(Node):
     
     def __init__(self):
         super().__init__("angle_publish")
         self.angle_pub = self.create_publisher(Float32,"servo_angle", 10)
         self.create_timer(1.0, self.publish_angle)
         self.default_angle = 0


     def publish_angle(self):
        msg = Float32()
        msg.data = self.default_angle
        self.angle_pub.publish(msg)
        self.get_logger().info("Angle:")
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = anglePublisher()
    rclpy.spin(node)
 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
