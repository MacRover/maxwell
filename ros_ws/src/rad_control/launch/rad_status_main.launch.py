from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Tad bit overkill
    # STATUS 1 READER NODE
    reader1_node = Node(
        package="spidercan",
        executable="reader.py",
        name="reader",
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
        name="reader",
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
        name="reader",
        parameters=[{
            "topic": "/can/rad_can_in",
            "channel": "can0",
            "can_id": 0xF00,
            "can_mask": 0x1FFFFFE0
        }]
    )

    rad1_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad1_status_node",
        parameters=[{
            "motor_id": 1,
            "namespace": "front_right",
            "status_rate": 10
        }]
    )
    rad2_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad1_status_node",
        parameters=[{
            "motor_id": 2,
            "namespace": "back_right",
            "status_rate": 10
        }]
    )
    rad3_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad1_status_node",
        parameters=[{
            "motor_id": 3,
            "namespace": "back_left",
            "status_rate": 10
        }]
    )
    rad4_node = Node(
        package="rad_control",
        executable="rad_status",
        name="rad1_status_node",
        parameters=[{
            "motor_id": 4,
            "namespace": "front_left",
            "status_rate": 10
        }]
    )

    return LaunchDescription(
        [
            reader1_node, reader2_node, reader3_node,
            rad1_node, rad2_node, rad3_node, rad4_node,
        ]
    )