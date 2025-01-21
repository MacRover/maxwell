from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {"autorepeat_rate": 20.0},  # can be changed if buffer starts, however servo will be jittery
        ],
    )

    dpad_node = Node(
        package="angle_publisher",  
        executable="controller_input.py",  
        name="controller_angle_controller"
    )
    return LaunchDescription(
        [
            joy_node,
            dpad_node,
        ]
    )
