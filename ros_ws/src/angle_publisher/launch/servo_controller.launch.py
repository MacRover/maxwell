from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    joy_rate = LaunchConfiguration("joy_rate")

    joy_rate_launch_arg = DeclareLaunchArgument(
        "joy_rate",
        default_value="10.0",
    )

    # Define the Joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {"autorepeat_rate": joy_rate}, # set in hz 
        ],
    )

    # Define the D-pad controller node
    dpad_node = Node(
        package="angle_publisher",
        executable="controller_input.py",
        name="controller_angle_controller"
    )

    return LaunchDescription([
        joy_rate_launch_arg,  
        joy_node,            
        dpad_node
        ])
    
