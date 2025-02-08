import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    params1_path = os.path.join(get_package_share_directory('usb_cam_config'), 'config', 'params1.yaml')

    params2_path = os.path.join(get_package_share_directory('usb_cam_config'), 'config', 'params2.yaml')

    usb_cam_0_enabled_arg = DeclareLaunchArgument(
        'usb_cam_0_enabled', default_value='true',
        description='Enable usb_cam_0 (true or false)'
    )

    usb_cam_1_enabled_arg = DeclareLaunchArgument(
        'usb_cam_1_enabled', default_value='true',
        description='Enable usb_cam_1 (true or false)'
    )

    realsense_enabled_arg = DeclareLaunchArgument(
        'realsense_enabled', default_value='true',
        description='Enable RealSense Camera (true or false)'
    )

    #usb_cam_0
    usb_cam_0_node = Node(
        package = "usb_cam",
        executable = "usb_cam_node_exe",
        name = "usb_cam_0_node",
        namespace = "usb_cam_0",
        parameters = [params1_path],
        condition = IfCondition(LaunchConfiguration('usb_cam_0_enabled'))
    )

    #usb_cam_1
    usb_cam_1_node = Node(
        package = "usb_cam",
        executable = "usb_cam_node_exe",
        name = "usb_cam_1_node",
        namespace = "usb_cam_1",
        parameters = [params2_path],
        condition = IfCondition(LaunchConfiguration('usb_cam_1_enabled'))
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ]),
        condition=IfCondition(LaunchConfiguration('realsense_enabled'))
    )

    return LaunchDescription(
        usb_cam_0_enabled_arg,
        usb_cam_1_enabled_arg,
        realsense_enabled_arg,
        usb_cam_0_node,
        usb_cam_1_node,
        realsense_launch
    )