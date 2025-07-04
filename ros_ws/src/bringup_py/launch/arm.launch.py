import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (RegisterEventHandler, LogInfo, 
                            IncludeLaunchDescription, DeclareLaunchArgument)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

rad_status_to_id = {
    "/arm/joint0/rad_status": 0x15,
    "/arm/joint1/rad_status": 0x16,
    "/arm/joint2/rad_status": 0x17,
}

def generate_launch_description():
    is_standalone_arg = DeclareLaunchArgument(
        "is_standalone",
        default_value="True",
        description="Launch as standalone script"
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
    is_standalone = LaunchConfiguration("is_standalone")
    can_rate = LaunchConfiguration("can_rate")
    can_channel = LaunchConfiguration("can_channel")

    rad_status_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rad_control"),
                "launch"
            ),
            "/rad_status_main.launch.py"
        ]),
        condition=IfCondition(is_standalone),
    )

    arm_controller_node = Node(
        package="rad_control",
        executable="rad_arm_controller",
        name="rad_arm_controller",
        namespace="arm",
        output="screen",
        parameters=[{
            #Can do the parameters from here if we want to configure them at launch
            "can_rate": can_rate,
        }]
    )

    wrist_calibration_node = Node(
        package="rad_control",
        executable="rad_calibration_wrist",
        name="rad_wrist_init",
        namespace="arm",
        output="screen"
    )

    joint_calibration_node = Node(
        package="rad_control",
        executable="rad_calibration_init",
        name="rad_arm_calibration_init",
        namespace="arm",
        parameters=[{
            "rad_ids":    list(rad_status_to_id.values()),
            "rad_status": list(rad_status_to_id.keys())
        }],
        output="screen"
    )

    rad_converter_node = Node(
        package="arm",
        executable="rad_converter",
        name="rad_converter",
        namespace="arm",
        output="screen"
    )

    xbox_servo_node = Node(
        package="arm",
        executable="joy_controller",
        name="joy_controller",
        namespace="arm",
        output="screen"
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("arm"),
                "launch"
            ),
            "/servo.launch.py"
        ]),
    )

    can_writer_node = Node(
        package="spidercan",
        executable="writer.py",
        name="writer",
        namespace="arm",
        parameters=[{
            "channel": can_channel,
            "raw_topic": "/can/can_out"
        }],
        condition=IfCondition(is_standalone),
    )

    main_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_calibration_node,
            on_exit=[
                arm_controller_node,
                servo_launch,
                xbox_servo_node
            ]
        )
    )
    calibration_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wrist_calibration_node,
            on_exit=[
                joint_calibration_node
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(is_standalone_arg)
    ld.add_action(can_rate_arg)
    ld.add_action(can_channel_arg)

    ld.add_action(rad_status_main)
    ld.add_action(wrist_calibration_node)
    ld.add_action(calibration_event)
    ld.add_action(main_event)
    ld.add_action(can_writer_node)
    # ld.add_action(rad_converter_node)

    return ld 