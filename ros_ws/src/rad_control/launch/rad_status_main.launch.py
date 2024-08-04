from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # STATUS READER NODE
    reader1_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad1_reader",
        parameters=[{
            "topic": "/can/rad_can_in_1",
            "channel": "can0",
            "can_ids": [0x900],
            "can_masks": [0x1FFFFFE0]
        }]
    )

    reader2_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad1_reader",
        parameters=[{
            "topic": "/can/rad_can_in_2",
            "channel": "can0",
            "can_ids": [0xE00],
            "can_masks": [0x1FFFFFE0]
        }]
    )

    reader3_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad1_reader",
        parameters=[{
            "topic": "/can/rad_can_in_3",
            "channel": "can0",
            "can_ids": [0xF00],
            "can_masks": [0x1FFFFFE0]
        }]
    )

    rad_status_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad_status_node",
        parameters=[{
            "can_topic_1": "/can/rad_can_in_1",
            "can_topic_2": "/can/rad_can_in_2",
            "can_topic_3": "/can/rad_can_in_3",
            "status_rate": 10
        }]
    )

    return LaunchDescription(
        [
            reader1_node, reader2_node, reader3_node,
            rad_status_node,
        ]
    )