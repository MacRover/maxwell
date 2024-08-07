from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    reader_node = Node(
        package="spidercan",
        executable="reader.py",
        name="vesc_reader",
        parameters=[{
            "topic": "/can/vesc_can_in",
            "channel": "can0",
            "can_ids": [0x950],
            "can_masks": [0x1FFFFFC0]
        }]
    )

    vesc1_node = Node(
        package="drive",
        executable="vesc_status.py",
        name="vesc1_status_node",
        parameters=[{
            "status": "STATUS_1",
            "motor": "BACK_RIGHT",
            "namespace": "rear_right",
            "status_rate": 15,
            "logging": True
        }]
    )

    vesc2_node = Node(
        package="drive",
        executable="vesc_status.py",
        name="vesc2_status_node",
        parameters=[{
            "status": "STATUS_1",
            "motor": "FRONT_RIGHT",
            "namespace": "front_right",
            "status_rate": 15,
            "logging": True
        }]
    )

    vesc4_node = Node(
        package="drive",
        executable="vesc_status.py",
        name="vesc4_status_node",
        parameters=[{
            "status": "STATUS_1",
            "motor": "FRONT_LEFT",
            "namespace": "front_left",
            "status_rate": 15,
            "logging": True
        }]
    )

    vesc3_node = Node(
        package="drive",
        executable="vesc_status.py",
        name="vesc3_status_node",
        parameters=[{
            "status": "STATUS_1",
            "motor": "BACK_LEFT",
            "namespace": "rear_left",
            "status_rate": 15,
            "logging": True
        }]
    )

    return LaunchDescription(
        [
            reader_node, 
            vesc1_node, vesc2_node,
            vesc3_node, vesc4_node
        ]
    )