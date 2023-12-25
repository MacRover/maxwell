import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

foxglove_launch = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(
        [
            os.path.join(get_package_share_directory("foxglove_bridge"), "launch"),
            "/foxglove_bridge_launch.xml",
        ]
    )
)


def generate_launch_description():
    return LaunchDescription(
        [
            foxglove_launch,
        ]
    )
