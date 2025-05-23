from launch import LaunchDescription
from launch_ros.actions import Node

rad_status_to_id = {
    "/drive/front_right/rad_status": 0x11,
    "/drive/front_left/rad_status": 0x12,
    "/drive/rear_left/rad_status": 0x13,
    "/drive/rear_right/rad_status": 0x14,
    "/arm/joint0/rad_status": 0x16,
    "/arm/joint1/rad_status": 0x15,
    "/arm/joint2/rad_status": 0x17,
    "/arm/wrist_rs/rad_status": 0x18,
    "/arm/wrist_ls/rad_status": 0x19,
}

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
            "status_rate": 15,
            "rad_status": list(rad_status_to_id),
            "rad_ids": list(rad_status_to_id.values())
        }]
    )

    return LaunchDescription(
        [
            reader1_node,
            reader2_node,
            rad_status_node,
        ]
    )