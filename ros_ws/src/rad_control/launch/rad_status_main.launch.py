from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # STATUS READER NODE
    reader1_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad1_reader",
        namespace="drive",
        parameters=[{
            "topic": "/can/status/rad_can_in",
            "channel": "can0",
            "can_ids": [0x404FB00, 0x404FC00],
            "can_masks": [0xFFFFF00, 0xFFFFF00]
        }]
    )

    # CONFIG READER NODE
    reader2_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad2_reader",
        namespace="drive",
        parameters=[{
            "topic": "/can/config/rad_can_in",
            "channel": "can0",
            "can_ids": [0x4040000],
            "can_masks": [0xFFF8000]
        }]
    )

    rad_status_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad_status_node",
        namespace="drive",
        parameters=[{
            "can_topic": "/can/status/rad_can_in",
            "status_rate": 15
        }]
    )

    return LaunchDescription(
        [
            reader1_node,
            reader2_node,
            rad_status_node,
        ]
    )