#! /usr/bin/env python
from math import pi
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from custom_interfaces.msg import SwerveModulesList

from .model import SteeringModel
from .drive_module import DriveModule


class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.publisher_modules_command = self.create_publisher(SwerveModulesList, "/modules_command", 10)
        self.subscription = self.create_subscription(
            SwerveModulesList,
            "/drive_modules",
            self.callback,
            10,
        )
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            "/cmd_vel_repeat",
            self.publishModulesCommand,
            10,
        )
        ## 0.54, 0.85, half: 0.27, 0.42
        modules = [
            DriveModule("front_left", (-0.27, 0.42), 1.0, pi),
            DriveModule("front_right", (0.27, 0.42), 1.0, pi),
            DriveModule("rear_left", (-0.27, -0.42), 1.0, pi),
            DriveModule("rear_right", (0.27, -0.42), 1.0, pi)
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
    
    def publishModulesCommand(self, msg):
        print("Received command %s" % msg)
        self.model.body_state = [msg.linear.x, msg.linear.y, msg.angular.z]
        out = SwerveModulesList()
        modules = self.model.getDriveModuleVelocities()
        out.front_left = modules[0]
        out.front_right = modules[1]
        out.rear_left = modules[2]
        out.rear_right = modules[3]
        print("Publishing command %s" % out)
        self.publisher_modules_command.publish(out)

def main(args=None):
    rclpy.init(args=args)
    print("Starting drive controller")
    node = DriveController()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
