from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    drive_mode_arg = DeclareLaunchArgument(
        "drive_mode",
        default_value="SWERVE_DRIVE" 
    )
    drive_mode = LaunchConfiguration("drive_mode")

    heartbeat_node = Node(
        package="drive",
        executable="heartbeat.py",
        name="heartbeat_node"
    )

    xbox_node = Node(
        package="drive",
        executable="xbox_drive.py",
        name="xbox_controller",
        parameters=[{
            "drive_mode": drive_mode
        }]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node"
    )

    rover_steer_node = Node(
        package="rad_control",
        executable="rover_steer_pos",
        name="steer_node",
        condition=LaunchConfigurationEquals(
            "drive_mode",
            "TANK_STEER_HYBRID"
        )
    )

    return LaunchDescription(
        [
            drive_mode_arg,
            heartbeat_node,
            joy_node, 
            xbox_node,
            rover_steer_node
        ]
    )