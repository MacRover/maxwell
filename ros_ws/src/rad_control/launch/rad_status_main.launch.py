from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Tad bit overkill
    # STATUS 1 READER NODE
    reader1_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad1_reader",
        parameters=[{
            "topic": "/can/rad_can_in",
            "channel": "can0",
            "can_id": 0x900,
            "can_mask": 0x1FFFFFE0
        }]
    )
    # STATUS 2 READER NODE
    reader2_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad2_reader",
        parameters=[{
            "topic": "/can/rad_can_in",
            "channel": "can0",
            "can_id": 0xE00,
            "can_mask": 0x1FFFFFE0
        }]
    )
    # STATUS 3 READER NODE
    reader3_node = Node(
        package="spidercan",
        executable="reader.py",
        name="rad3_reader",
        parameters=[{
            "topic": "/can/rad_can_in",
            "channel": "can0",
            "can_id": 0xF00,
            "can_mask": 0x1FFFFFE0
        }]
    )

    rad_status_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad_status_node",
        parameters=[{
            "can_topic": "/can/rad_can_in",
            "status_rate": 10
        }]
    )

    return LaunchDescription(
        [
            reader1_node, reader2_node, reader3_node,
            rad_status_node,
        ]
    )