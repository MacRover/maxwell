import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (RegisterEventHandler, LogInfo, 
                            IncludeLaunchDescription, DeclareLaunchArgument)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals

rad_status_to_id = {
    "/front_right/rad_status": 0x11,
    "/front_left/rad_status": 0x12,
    "/rear_left/rad_status": 0x13,
    "/rear_right/rad_status": 0x14
}

def generate_launch_description():
    drive_mode_arg = DeclareLaunchArgument(
        "drive_mode",
        default_value="SWERVE_DRIVE",
        description="Method of Driving (SWERVE_DRIVE | TANK_STEER_HYBRID)"
    )
    can_rate_arg = DeclareLaunchArgument(
        "can_rate",
        default_value="10",
        description="Rate of CAN Messages (hz)"
    )
    can_channel_arg = DeclareLaunchArgument(
        "can_channel",
        default_value="can0",
        description="socketCAN channel"
    )
    vesc_enabled_arg = DeclareLaunchArgument(
        "vesc_enabled",
        default_value="True",
        description="Enable VESC controller node for drive"
    )
    wait_until_positioned_arg = DeclareLaunchArgument(
        "wait_until_positioned",
        default_value="True",
        description="Enable drive only when RAD motors are positioned at setpoint (for swerve drive)"
    )
    drive_mode = LaunchConfiguration("drive_mode")
    can_rate = LaunchConfiguration("can_rate")
    vesc_enabled = LaunchConfiguration("vesc_enabled")
    wait_until_positioned = LaunchConfiguration("wait_until_positioned")
    can_channel = LaunchConfiguration("can_channel")


    rad_status_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rad_control"),
                "launch"
            ),
            "/rad_status_main.launch.py"
        ])
    )
    vesc_status_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("drive"),
                "launch"
            ),
            "/vesc_status.launch.py"
        ])
    )

    drive_controller_node = Node(
        package="drive",
        executable="drive_controller.py",
        name="drive_controller",
        namespace="drive",
        parameters=[{
            "drive_mode": drive_mode
        }]
    )

    odom_controller_node = Node(
        package="drive",
        executable="drive_modules_odom.py",
        name="drive_modules_odom",
        namespace="drive",
    )

    vesc_controller_node = Node(
        package="drive",
        executable="vesc_controller.py",
        name="vesc_controller",
        namespace="drive",
        parameters=[{
            "can_rate": can_rate,
            "wait_until_positioned": wait_until_positioned 
            # Note: must be set to False when in open loop/pulse control
        }],
        condition=IfCondition(vesc_enabled)
    )

    rad_controller_node = Node(
        package="rad_control",
        executable="rad_drive_controller",
        name="rad_drive_controller",
        namespace="drive",
        parameters=[{
            "can_rate": can_rate
        }]
    )

    rad_init_node = Node(
        package="rad_control",
        executable="rad_calibration_init",
        name="rad_drive_calibration_init",
        namespace="drive",
        parameters=[{
            "rad_ids":    list(rad_status_to_id.values()),
            "rad_status": list(rad_status_to_id.keys())
        }],
        condition=LaunchConfigurationEquals(
                "drive_mode",
                "SWERVE_DRIVE"
        )
    )

    can_writer_node = Node(
        package="spidercan",
        executable="writer.py",
        name="writer",
        namespace="drive",
        parameters=[{
            "channel": can_channel
        }]
    )

    rover_steer_node = Node(
        package="rad_control",
        executable="rover_steer_pos",
        name="steer_node",
        namespace="drive",
        condition=LaunchConfigurationEquals(
            "drive_mode",
            "TANK_STEER_HYBRID"
        ),
        parameters=[{
            "can_rate": can_rate
        }]
    )

    ld = LaunchDescription()
    ld.add_action(drive_mode_arg)
    ld.add_action(can_rate_arg)
    ld.add_action(vesc_enabled_arg)
    ld.add_action(wait_until_positioned_arg)
    ld.add_action(can_channel_arg)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=rad_init_node,
                on_exit=[
                    LogInfo(msg="Starting RAD CAN controller"),
                    rad_controller_node
                ]
            ),
            condition=LaunchConfigurationEquals(
                    "drive_mode",
                    "SWERVE_DRIVE"
            )
        )
    )
    ld.add_action(rad_init_node)

    ld.add_action(can_writer_node)

    ld.add_action(rad_status_main)
    ld.add_action(vesc_status_main)

    ld.add_action(drive_controller_node)
    ld.add_action(vesc_controller_node)
    ld.add_action(odom_controller_node)
    ld.add_action(rover_steer_node)

    return ld
