Launch the controller using:

```
ros2 run drive drive_controller.py
```

Controller listens for cmd_vel messages as input, and will output swerve module commands to topic `/modules_command`. It also publishes odom messages to `/odom` topic if data is received to `/drive_modules`.

Temporarily can use teleop controller that comes with ros2 to control rover. Launch using:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To control rover in simulation using cmd_vel messages, need to also launch node call sim_command_converter that converts `/modules_command` to the correct gazebo controller command with the angle aligned from the y-axis instead of x-axis. Launch using:
```
ros2 run drive sim_command_converter.py
```


#TODO

Allow launch arguments/create launch file with arguments for messages and wheel information