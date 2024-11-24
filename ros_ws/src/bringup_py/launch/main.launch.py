import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
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
    uros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["udp4", "-p", "9999"]
    )
    return LaunchDescription(
        [
            uros_agent_node,
            foxglove_launch,
        ]
    )
