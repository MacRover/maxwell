import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapviz',
            executable='mapviz'
            # remappings=[
            #     ('/fix', '/obc/gps')
            # ]
        ),
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            remappings=[
                ('/fix', '/obc/gps')
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
