# Maxwell Bringup
The following outlines the standard procedures to set up maxwell.
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
