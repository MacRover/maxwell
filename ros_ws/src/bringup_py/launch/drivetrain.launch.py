from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    heartbeat_node = Node(
        package="drive",
        executable="heartbeat.py",
        name="heartbeat_node"
    )

    drive_controller_node = Node(
        package="drive",
        executable="drive_controller.py",
        name="drive_controller"
    )

    vesc_controller_node = Node(
        package="drive",
        executable="vesc_controller.py",
        name="vesc_controller"
    )

    can_writer_node = Node(
        package="spidercan",
        executable="writer.py",
        name="writer"
    )

    return LaunchDescription(
        [
            heartbeat_node,
            drive_controller_node,
            vesc_controller_node,
            can_writer_node,
        ]
    )