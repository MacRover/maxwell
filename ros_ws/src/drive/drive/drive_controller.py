#! /usr/bin/env python
from math import pi, sin, cos
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from custom_interfaces.msg import SwerveModulesList, SwerveModule

from .model import SteeringModel
from .drive_module import DriveModule

class DriveMode(Enum):
	SWERVE_DRIVE = 0
	TANK_STEER_HYBRID = 1

class Drive:
    # Main callback function to /cmd_vel_repeat
    # Publishes to /modules_command
    def publishModulesCommand(self, msg: SwerveModulesList) -> None:
        pass
    # Takes encoder data from RAD and VESCs from /drive_modules and converts
    # it to Twist msg. Publish to Odom msg /odom
    def publishOdom(self, modules: SwerveModulesList, stamp: time) -> None:
        pass

class SwerveDrive(Drive):
    def __init__(self, pub_odom: Publisher, pub_modules: Publisher) -> None:
        self.pub_odom = pub_odom
        self.pub_modules = pub_modules
        ## 0.54, 0.85, half: 0.27, 0.42
        modules = [
            DriveModule("front_left", (-0.27, 0.42), 1.0, pi),
            DriveModule("front_right", (0.27, 0.42), 1.0, pi),
            DriveModule("rear_left", (-0.27, -0.42), 1.0, pi),
            DriveModule("rear_right", (0.27, -0.42), 1.0, pi)
        ]
        self.model = SteeringModel(modules)
    
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
        self.pub_modules.publish(out)
    
    def publishOdom(self, modules, stamp):
        body_state = self.model.updateOdom(modules)
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = "base_footprint"
        out.twist.twist.linear.x = body_state[0]
        out.twist.twist.linear.y = body_state[1]
        out.twist.twist.angular.z = body_state[2]
        self.pub_odom.publish(out)


class TankSteerDrive(Drive):
    def __init__(self, pub_odom: Publisher, pub_modules: Publisher) -> None:
        self.pub_odom = pub_odom
        self.pub_modules = pub_modules
        self.width = 0.7366
    
    def publishModulesCommand(self, msg):
        left_speed = msg.linear.x - msg.angular.z*self.width/2
        right_speed = msg.linear.x + msg.angular.z*self.width/2
        out = SwerveModulesList()
        out.front_left = SwerveModule(speed=left_speed)
        out.rear_left = SwerveModule(speed=left_speed)
        out.front_right = SwerveModule(speed=right_speed)
        out.rear_right = SwerveModule(speed=right_speed)
        print("Tank publishing command %s" % out)
        self.pub_modules.publish(out)
    
    def publishOdom(self, modules, stamp):
        left_speed = (modules.front_left.speed + modules.rear_left.speed) / 2.0
        right_speed = (modules.front_right.speed + modules.rear_right.speed) / 2.0
        lin = (left_speed + right_speed) / 2.0
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = "base_footprint"
        out.twist.twist.linear.x = lin*cos(modules.front_left.angle)
        out.twist.twist.linear.y = lin*sin(modules.front_left.angle)
        out.twist.twist.angular.z = (right_speed - left_speed) / self.width
        self.pub_odom.publish(out)
        

# ---------------------------------
# -- Begin Drive Controller Node --
# ---------------------------------
class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.publisher_modules_command = self.create_publisher(SwerveModulesList, "/modules_command", 10)
        self.drive: Drive = SwerveDrive(self.publisher, self.publisher_modules_command)
        self.subscription = self.create_subscription(
            SwerveModulesList,
            "/drive_modules",
            self.callback,
            10,
        )
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            "/cmd_vel_repeat",
            self.drive.publishModulesCommand,
            10,
        )
    
    def callback(self, msg):
        self.get_logger().info("Received message %s" % msg)
        self.drive.publishOdom(msg.modules, rclpy.clock.Clock().now().to_msg())

def main(args=None):
    rclpy.init(args=args)
    print("Starting drive controller")
    node = DriveController()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
