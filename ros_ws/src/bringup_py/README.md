# Maxwell Bringup
The following outlines the standard procedures to set up maxwell.
## Main Way
Main launch script for maxwell:
```bash
ros2 launch bringup_py maxwell_main.launch.py
```
Main launch script for basestation:
```bash
ros2 launch bringup_py bs_main.launch.py
```

The parameters used by these launch files (by default) are stored in `config/defaults.yaml`. They can be changed dynamically and automatically as long as you built the workspace with `--symlink-install`. All you have to do then is rerun these scripts. 

Otherwise, rebuild the workspace with `colcon build` every time `defaults.yaml` is changed and rerun the script(s) for it to take effect.

### Arguments
| **Argument**              | **Description**   | **Value Type**   |
| :-----------------------: | :---------------: | :-----------------: |
| `foxglove_enabled`        | Enable websocket for foxglove (foxglove bridge) | `True` or `False` |
| `microROS_enabled`        | Enable micro-ROS agent | `True` or `False` |
| `drive_enabled`           | Enable drivetrain (launch `drivetrain.launch.py`)    | `True` or `False` |
| `port_number`             | UDP4 port number for micro-ROS | Integer
| `drive_mode`              | Current drive mode of rover | `SWERVE_DRIVE` or `TANK_STEER_HYBRID` |
| `wait_until_positioned`   | Enable drive motors only when RAD motors are positioned at setpoint (`SWERVE_DRIVE` only) | `True` or `False` |
| `vesc_controller_enabled` | Enable `vesc_controller` node to control VESCs | `True` or `False` |
| `can_rate`                | Rate of CAN transmission per device (RAD and VESC) | Integer (in Hz) |
| `can_channel`             | socketCAN channel | `can0`, `can1`, `can2`, ... |

**NOTE**, ensure that arguments `drive_mode` and `can_rate` are the same value across Orin and basestation WS.

## Drive
The drive can be configured in two different modes:
- `SWERVE_DRIVE` (default)
- `TANK_STEER_HYBRID`

https://github.com/MacRover/maxwell/blob/7391d0cfab4a848145f715b26188028c4993c14e/ros_ws/src/drive/drive/drive_controller.py#L17-L19
As defined in `drive_controller.py`.

To launch the drive, run the following command on the Orin:
```bash
# <DRIVE_MODE> can either be SWERVE_DRIVE or TANK_STEER_HYBRID
ros2 launch bringup_py drivetrain.launch.py drive_mode:=<DRIVE_MODE>
```
Then, on another machine (basestation), run:
```bash
ros2 launch drive xbox_controller.launch.py drive_mode:=<DRIVE_MODE>
```

## micro-ROS
Running the micro-ROS agent separately:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 9999
```
