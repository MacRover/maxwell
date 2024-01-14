#! /usr/bin/env python
from math import pi
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from drive_msg.msg import SwerveModulesList

from .model import SteeringModel
from .drive_module import DriveModule


class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.subscription = self.create_subscription(
            SwerveModulesList,
            "/drive_modules",
            self.callback,
            10,
        )
        modules = [
            DriveModule("front_left", (0.5, 0.5), 1.0, 2.0),
            DriveModule("front_right", (0.5, -0.5), 1.0, 2.0),
            DriveModule("rear_left", (-0.5, 0.5), 1.0, 2.0),
            DriveModule("rear_right", (-0.5, -0.5), 1.0, 2.0)
        ]
        self.model = SteeringModel(modules)
    
    ## TODO publish odometry at regular intervals
    def publishOdom(self, modules):

        body_state = self.model.updateOdom(modules)

        out = Odometry()
        out.header.stamp = rclpy.clock.Clock().now().to_msg()
        out.header.frame_id = "base_footprint"
        out.twist.twist.linear.x = body_state[0]
        out.twist.twist.linear.y = body_state[1]
        out.twist.twist.angular.z = body_state[2]
        self.publisher.publish(out)

    def callback(self, msg):
        self.get_logger().info("Received message %s" % msg)
        self.publishOdom(msg.modules)

def main(args=None):
    rclpy.init(args=args)

    node = DriveController()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
