from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
      Node(
          package='rad_control',
          executable='rover_arm_test',
          name='rover_arm_test'
      )
  ])