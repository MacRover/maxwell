import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output='screen',
            remappings=[
                ('/obc/gps', '/navsat/fix')
            ]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
