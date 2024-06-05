from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "robot_name",
        default_value="robot_description",
        description="Ignition model name",
    ),
    DeclareLaunchArgument(
        "world", default_value="empty_world", description="World name"
    ),
]


def generate_launch_description():
    namespace = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="clock_bridge",
        output="screen",
        arguments=["/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock"],
        condition=IfCondition(use_sim_time),
    )

    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cmd_vel_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            [
                "/model/",
                LaunchConfiguration("robot_name"),
                "/cmd_vel" + "@geometry_msgs/msg/Twist" + "]ignition.msgs.Twist",
            ],
        ],
        remappings=[
            (
                ["/model/", LaunchConfiguration("robot_name"), "/cmd_vel"],
                "cmd_vel_unstamped",
            )
        ],
    )

    # Pose bridge
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="pose_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            [
                "/model/",
                LaunchConfiguration("robot_name"),
                "/pose" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            ],
            [
                "/model/",
                LaunchConfiguration("robot_name"),
                "/pose_static" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            ],
        ],
        remappings=[
            (
                ["/model/", LaunchConfiguration("robot_name"), "/pose"],
                "sim_ground_truth_pose",
            ),
            (
                ["/model/", LaunchConfiguration("robot_name"), "/pose_static"],
                "sim_ground_truth_pose_static",
            ),
        ],
    )

    # tf bridge
    odom_base_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="odom_base_tf_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            [
                "/model/",
                LaunchConfiguration("robot_name"),
                "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            ]
        ],
        remappings=[(["/model/", LaunchConfiguration("robot_name"), "/tf"], "/tf")],
    )

    # rgbd camera bridge
    rgbd_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='rgbd_camera_bridge',
        parameters=[{
            'use_sim_time': use_sim_time
            }],
        arguments=[
            ['/intel_realsense/camera_info' + '@sensor_msgs/msg/CameraInfo'  + '[ignition.msgs.CameraInfo'],
            ['/intel_realsense' + '@sensor_msgs/msg/Image'       + '[ignition.msgs.Image'],
            ['/intel_realsense/points'      + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked']
        ]
    )

    # IMU bridge
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="imu_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time
        }],
        arguments=[
            ['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/chassis/sensor/imu_sensor/imu' +
             "@sensor_msgs/msg/Imu[ignition.msgs.IMU"]
        ],
        remappings=[
            (['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/chassis/sensor/imu_sensor/imu'],
              "/imu")
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(clock_bridge)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(pose_bridge)
    ld.add_action(odom_base_tf_bridge)
    ld.add_action(rgbd_camera_bridge)
    ld.add_action(imu_bridge)
    return ld
