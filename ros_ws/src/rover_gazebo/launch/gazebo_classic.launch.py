import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    pkg_bringup = get_package_share_directory("rover_gazebo")
    pkg_robot_description = get_package_share_directory("robot_description")

    # Paths
    robot_description_base_launch = PathJoinSubstitution(
        [pkg_robot_description, "launch", "base.launch.py"]
    )
    robot_description_controller_launch = PathJoinSubstitution(
        [pkg_robot_description, "launch", "controllers.launch.py"]
    )

    # Launch configurations
    x, y, z = (
        LaunchConfiguration("x"),
        LaunchConfiguration("y"),
        LaunchConfiguration("z"),
    )
    yaw = LaunchConfiguration("yaw")

    # Robot description
    robot_description_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_base_launch]),
        launch_arguments=[("use_sim_time", "true")],
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'Maxwell'],
        output='screen',
        parameters=[{'use_sim_time': True}]  # Change from 'true' (string) to True (boolean)
    )

    # For the sim
    gazebo_params_file = os.path.join(get_package_share_directory('rover_gazebo'),'config','gazebo_params.yaml')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items(),
    )

    # Rviz2 launch 
    rviz_config_path = os.path.join(get_package_share_directory('rover_gazebo'),'config','maxwell_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    # Delay launch of the controllers until after Ignition has launched and the model
    # has spawned in Ignition because the Ignition controller plugins don't become available
    # until the model has spawned
    robot_controllers = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([robot_description_controller_launch]),
                launch_arguments=[
                    ("use_sim_time", "true"),
                    ("use_fake_hardware", "false"),
                    ("fake_sensor_commands", "false"),
                ],
            )
        ],
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(robot_description_base)
    ld.add_action(robot_controllers)
    ld.add_action(rviz_node)

    return ld
