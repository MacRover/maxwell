import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import FrontendLaunchDescriptionSource,PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

def generate_default_params() -> dict:
    config_path = os.path.join(get_package_share_directory("bringup_py"), "config", "defaults.yaml")
    with open(config_path, "r") as p:
        params = yaml.safe_load(p)
    return params


def generate_launch_description():
    dp = generate_default_params()

    microros_launch_arg = DeclareLaunchArgument(
        "microROS_enabled",
        default_value=str(dp["microROS_enabled"])
    )
    drive_enabled_launch_arg = DeclareLaunchArgument(
        "drive_enabled",
        default_value=str(dp["drive_enabled"])
    )
    arm_enabled_launch_arg = DeclareLaunchArgument(
        "arm_enabled",
        default_value=str(dp["arm_enabled"])
    )
    foxglove_enabled_launch_arg = DeclareLaunchArgument(
        "foxglove_enabled",
        default_value=str(dp["foxglove_enabled"])
    )
    microros = LaunchConfiguration("microROS_enabled")
    drive_enabled = LaunchConfiguration("drive_enabled")
    arm_enabled = LaunchConfiguration("arm_enabled")
    foxglove_enabled = LaunchConfiguration("foxglove_enabled")

    uros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["udp4", "-p", str(dp["microROS"]["port_number"])],
        condition=IfCondition(microros)
    )

    foxglove_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("foxglove_bridge"), "launch"),
                "/foxglove_bridge_launch.xml",
            ]
        ),
        condition=IfCondition(foxglove_enabled)
    )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("bringup_py"), "launch"),
                "/arm.launch.py"
            ]
        ),
        launch_arguments={
            "is_standalone": "False",
            "can_rate": str(dp["arm"]["can_rate"]),
            "can_channel": str(dp["arm"]["can_channel"])
        }.items(),
        condition=IfCondition(arm_enabled)
    )

    drivetrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("bringup_py"), "launch"),
                "/drivetrain.launch.py"
            ]
        ),
        launch_arguments={
            "drive_mode": dp["drive"]["drive_mode"],
            "can_rate": str(dp["drive"]["can_rate"]),
            "vesc_enabled": str(dp["drive"]["vesc_controller_enabled"]),
            "wait_until_positioned": str(dp["drive"]["wait_until_positioned"]),
            "can_channel": dp["drive"]["can_channel"]
        }.items(),
        condition=IfCondition(drive_enabled)
    )

    return LaunchDescription(
        [
            microros_launch_arg,
            drive_enabled_launch_arg,
            arm_enabled_launch_arg,
            foxglove_enabled_launch_arg,
            uros_agent_node,
            foxglove_launch,
            drivetrain_launch,
            arm_launch,
        ]
    )
