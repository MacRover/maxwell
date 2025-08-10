import os
import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_default_params() -> dict:
    config_path = os.path.join(get_package_share_directory("bringup_py"), "config", "defaults.yaml")
    with open(config_path, "r") as p:
        params = yaml.safe_load(p)
    return params


def generate_launch_description():
    dp = generate_default_params()

    xbox_controller_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("drive"), "launch"),
            "/xbox_controller.launch.py"
        ]),
        launch_arguments={
            "drive_mode": dp["drive"]["drive_mode"],
            "can_rate": str(dp["drive"]["can_rate"]),
        }.items()
    )

    xbox_controller_1 = Node(
        package="joy",
        executable="joy_node",
        name="arm_joy_node",
        parameters=[{
            "device_id": 1,
        }],
        remappings=[
            ("joy", "joy1")
        ],
    )

    arm_joy_controller = Node(
        package="arm",
        executable="joy_controller",
        name="arm_xbox_controller",
    )

    ld = LaunchDescription()
    ld.add_action(xbox_controller_0)
    ld.add_action(xbox_controller_1)
    ld.add_action(arm_joy_controller)
    return ld
