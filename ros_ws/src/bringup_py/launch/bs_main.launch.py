import os
import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_default_params() -> dict:
    config_path = os.path.join(get_package_share_directory("bringup_py"), "config", "defaults.yaml")
    with open(config_path, "r") as p:
        params = yaml.safe_load(p)
    return params


def generate_launch_description():
    dp = generate_default_params()

    xbox_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("drive"), "launch"),
            "/xbox_controller.launch.py"
        ]),
        launch_arguments={
            "drive_mode": dp["drive"]["drive_mode"],
            "can_rate": str(dp["drive"]["can_rate"]),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(xbox_controller)
    return ld
