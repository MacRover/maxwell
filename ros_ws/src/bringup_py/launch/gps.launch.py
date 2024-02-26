import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('bringup_py'),
        'config')
    params = os.path.join(config_directory, 'neom9n.yaml')
    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
                                             executable='ublox_gps_node',
                                             output='both',
                                             remappings=[
                                                ('/fix', '/ublox/fix'),
                                                ('/fix_velocity', '/ublox/fix_velocity')
                                             ],
                                             parameters=[params])

    return launch.LaunchDescription([ublox_gps_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])