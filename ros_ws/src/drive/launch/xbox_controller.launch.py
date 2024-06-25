from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    heartbeat_node = Node(
        package="drive",
        executable="heartbeat.py",
        name="heartbeat_node"
    )

    xbox_node = Node(
        package="drive",
        executable="xbox_drive.py",
        name="xbox_controller"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node"
    )

    return LaunchDescription(
        [
            heartbeat_node,
            joy_node, 
            xbox_node,
        ]
    )