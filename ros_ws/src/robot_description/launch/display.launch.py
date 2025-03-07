from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():

    pkg_robot_description = get_package_share_path("robot_description")
    model_path = pkg_robot_description / "urdf/robot_description.urdf"
    rviz_config_path = pkg_robot_description / "config/urdf.rviz"

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(model_path),
        description="Absolute path to robot urdf file",
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(rviz_config_path),
        description="Absolute path to rviz config file",
    )
    is_simulation_arg = DeclareLaunchArgument(
        name="is_simulation",
        default_value="false",
        choices=["true", "false"],
        description="Is simulation or not",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model"), " is_simulation:=", LaunchConfiguration("is_simulation")]), value_type=str
    )
    # doc = xacro.process_file(str(model_path), mappings={"is_simulation": is_simulation})

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            gui_arg,
            model_arg,
            rviz_arg,
            is_simulation_arg,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
