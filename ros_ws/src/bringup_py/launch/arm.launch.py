import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    arm_controller_node = Node(
        package="rad_control",
        executable="rad_arm_controller",
        name="rad_arm_controller",
        namespace="arm",
        parameters=[{
            #Can do the parameters from here if we want to configure them at launch
        }]
    )

    wrist_controller_node = Node(
        package="rad_control",
        executable="rad_calibration_wrist",
        name="rad_wrist_init",
        namespace="arm"
    )

    arm_hardware_node = Node(
        package="arm_controller",
        executable="arm_hardware",
        name="arm_hardware",
        namespace="arm"
    )

    rad_converter_node = Node(
        package="arm_controller",
        executable="rad_converter",
        name="rad_converter",
        namespace="arm"
    )

    Xbox_servo_node = Node(
        package="arm_controller",
        executable="joy_controller",
        name="joy_controller",
        namespace="arm"
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("arm_controller"),
                "launch"
            ),
            "/servo.launch.py"
        ])
    )




    ld = LaunchDescription()
    ld.add_action(arm_controller_node)
    ld.add_action(wrist_controller_node)
    ld.add_action(arm_hardware_node)
    ld.add_action(servo.launch)
    ld.add_action(rad_converter_node)
    ld.add_action(xbox_servo_node)

    return ld 