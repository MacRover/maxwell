from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # STATUS READER NODE
    reader1_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad1_reader",
        parameters=[{
            "topic": "/can/rad_can_in",
            "channel": "can0",
            "can_ids": [0x404FB00, 0x404FC00],
            "can_masks": [0x1FFFFFF0, 0x1FFFFFF0]
        }]
    )

    rad_status_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad_status_node",
        parameters=[{
            "can_topic": "/can/rad_can_in",
            "status_rate": 15
        }]
    )

    return LaunchDescription(
        [
            reader1_node,
            rad_status_node,
        ]
    )