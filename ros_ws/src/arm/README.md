# Main Arm package

This package contains auxilary nodes, and launch files for operating the arm.

```
├── launch
│   ├── keyboard_control_launch.py # Launches joint based keyboard control
│   └── servo.launch.py            # Launches moveit servoing
└── src
    ├── arm_rad_dummy.cpp          # Rad dummy node used for testing when hardware isnt available
    ├── joy_controller.cpp         # Joy controller node used for servoing
    ├── keyboard_controller.cpp    # Joint based keyboard control node
    ├── rad_converter.cpp          # Converts rad_status to joint_states
    └── servo_keyboard_input.cpp   # Keyboard controller node used for servoing
```


Other packages that might be of interest:

```
arm_controller     ->  ros2 controller and hardware implementations
arm_moveit_config  ->  moveit configuration files for the arm (mostly auto-gen)
robot_description  ->  URDF for the arm can be found here
rad_control        ->  Code related to communicating to RADs is here
```


Regardless of the operating mode, these nodes have to be running to operate the arm:

```
# Convert rad messages to joint states
ros2 run arm rad_converter 

# Convert joint states to rad messages
ros2 run rad_control rad_arm_controller
```

For servoing, launch:
```
ros2 launch arm servo.launch.py
```
and either the joystick (`joy_controller`) or keyboard (`servo_keyboard_input`) nodes

The configuration file (`servo_config.yaml`) for servoing can be found in `/config` of the moveit package. Modify this for adjusting servoing behaviour!

Other configuration files such as initial position, joint limits, controller types can also be found in `/config` of the moveit package.




