from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    reader_node = Node(
        package="spidercan",
        executable="reader.py",
        name="reader",
        parameters=[{
            "channel": "can0",
            "can_id": 0x950,
            "can_mask": 0x1FFFFFC0
        }]
    )

    status_node = Node(
        package="drive",
        executable="vesc_status.py",
        name="vesc_status_node",
        parameters=[{
            "status": "STATUS_1",
            "motor": "FRONT_RIGHT"
        }]
    )

    return LaunchDescription(
        [
            reader_node, 
            status_node,
        ]
    )