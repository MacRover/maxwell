from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # STATUS READER NODE
    reader1_node = Node(
        package="spidercan",
        executable="reader.py",
        name="viper1_reader",
        parameters=[{
            "topic": "/can/status/viper_can_in",
            "channel": "can0",
            # IDs F0 - FF (240-255)
            "can_ids": [0xA04F000],
            "can_masks": [0xFFFF000]
        }]
    )

    # CONFIG READER NODE
    reader2_node = Node(
        package="spidercan",
        executable="reader.py",
        name="viper2_reader",
        parameters=[{
            "topic": "/can/config/viper_can_in",
            "channel": "can0",
            "can_ids": [0xA040000],
            "can_masks": [0xFFF8000]
        }]
    )

    viper_status_node = Node(
        package="viper_control",
        executable="viper_status",
        name="viper_status_node",
        parameters=[{
            "can_topic": "/can/status/viper_can_in",
            "status_rate": 15
        }]
    )

    return LaunchDescription(
        [
            reader1_node,
            reader2_node,
            viper_status_node,
        ]
    )