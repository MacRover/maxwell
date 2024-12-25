import os
import yaml
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

def generate_default_params() -> dict:
    config_path = os.path.join(get_package_share_directory("bringup_py"), "config", "defaults.yaml")
    with open(config_path, "r") as p:
        params = yaml.safe_load(p)
    return params


def generate_launch_description():
    dp = generate_default_params()

    return LaunchDescription()
