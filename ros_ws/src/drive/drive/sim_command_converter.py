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


class SimCommandConverter(Node):
    def __init__(self):
        super().__init__("sim_command_converter")
        self.publisher_swerve_ = self.create_publisher(
            Float64MultiArray, "/drive_module_steering_angle_controller/commands", 10
        )
        self.publisher_wheel_ = self.create_publisher(
            Float64MultiArray, "/drive_module_velocity_controller/commands", 10
        )
        self.subscription = self.create_subscription(
            SwerveModulesList,
            "/drive/modules_command",
            self.callback,
            10,
        )

    def callback(self, msg: SwerveModulesList):
      self.swerve_msg = Float64MultiArray()
      self.swerve_msg.data = [0.0, 0.0, 0.0, 0.0]
      self.wheel_msg = Float64MultiArray()
      self.wheel_msg.data = [0.0, 0.0, 0.0, 0.0]
      for i, mod in enumerate(msg.modules):
          #convert angle that starts from x axis to -pi to pi from y axis
          angle = mod.angle
          if angle > pi/2:
            angle = mod.angle - pi
          elif angle < -pi/2:
            angle = pi - angle

          self.swerve_msg.data[i] = angle
          self.wheel_msg.data[i] = mod.speed
      self.publisher_swerve_.publish(self.swerve_msg)
      self.publisher_wheel_.publish(self.wheel_msg)
    

def main(args=None):
    rclpy.init(args=args)
    print("Starting sim command converter")
    node = SimCommandConverter()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
