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
    can_rate_arg = DeclareLaunchArgument(
        "can_rate",
        default_value="10"
    )
    drive_mode = LaunchConfiguration("drive_mode")
    can_rate = LaunchConfiguration("can_rate")

    heartbeat_node = Node(
        package="drive",
        executable="heartbeat.py",
        name="heartbeat_node",
        parameters=[{
            "drive_mode": drive_mode,
            "topic_rate": can_rate
        }]
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

    return LaunchDescription(
        [
            drive_mode_arg,
            can_rate_arg,
            heartbeat_node,
            joy_node, 
            xbox_node,
        ]
    )