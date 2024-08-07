import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (RegisterEventHandler, LogInfo, TimerAction, 
                            IncludeLaunchDescription, DeclareLaunchArgument)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    drive_mode_arg = DeclareLaunchArgument(
        "drive_mode",
        default_value="SWERVE_DRIVE"
    )
    can_rate_arg = DeclareLaunchArgument(
        "can_rate",
        default_value="10"
    )

    rad_status_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rad_control"),
                "launch"
            ),
            "/rad_status_main.launch.py"
        ])
    )

    drive_controller_node = Node(
        package="drive",
        executable="drive_controller.py",
        name="drive_controller",
        parameters=[{
            "drive_mode": LaunchConfiguration("drive_mode")
        }]
    )

    vesc_controller_node = Node(
        package="drive",
        executable="vesc_controller.py",
        name="vesc_controller",
        parameters=[{
            "can_rate": LaunchConfiguration("can_rate")
        }]
    )

    rad_controller_node = Node(
        package="rad_control",
        executable="rad_drive_controller",
        name="rad_drive_controller",
        parameters=[{
            "can_rate": LaunchConfiguration("can_rate")
        }]
    )

    rad_init_node = Node(
        package="rad_control",
        executable="rad_calibration_init",
        name="rad_calibration_init"
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
    ld.add_action(drive_mode_arg)
    ld.add_action(can_rate_arg)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=rad_init_node,
                on_exit=[
                    LogInfo(msg="Starting CAN controllers"),
                    vesc_controller_node,
                    rad_controller_node
                ]
            )
        )
    )
    ld.add_action(rad_init_node)

    ld.add_action(can_writer_node)
    ld.add_action(rad_status_main)
    ld.add_action(drive_controller_node)
    # ld.add_action(vesc_controller_node)
    # ld.add_action(rad_controller_node)
    return ld
