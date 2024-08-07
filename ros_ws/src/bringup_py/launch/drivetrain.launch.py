from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    drive_controller_node = Node(
        package="drive",
        executable="drive_controller.py",
        name="drive_controller",
        parameters=[{
            "drive_mode": "SWERVE_DRIVE"
        }]
    )

    vesc_controller_node = Node(
        package="drive",
        executable="vesc_controller.py",
        name="vesc_controller"
    )

    can_writer_node = Node(
        package="spidercan",
        executable="writer.py",
        name="writer",
        parameters=[{
            "channel": "can0"
        }]
    )

    ld = LaunchDescription()
    ld.add_action(drive_controller_node)
    ld.add_action(vesc_controller_node)
    ld.add_action(can_writer_node)
    return ld
