# rover_gazebo

This is the simulation package for the rover. 

The following packages need to be installed for simulation to work:
```
sudo apt-get install ignition-fortress
```
```
sudo apt-get install ros-humble-ign-ros2-control
```

Build using at the root of the workspace, you might get a warning about setup.py which you can ignore:
```
colcon build
```

<br />

## Launch Simulation:
```
ros2 launch rover_gazebo ignition.launch.py
```

Launch the keyboard controller:
```
ros2 run keyboard_controller_sim keyboard_controller  
```

## Extra Info

The simulation uses virtual controllers for controlling each joint. You can control these joints by sending Float64MultiArray of size 4 to the following topics:

- Swerve joint: `/drive_module_steering_angle_controller/commands`
- Wheel joint: `/drive_module_velocity_controller/commands`

The keyboard controller is publishing to these topics to move the robot in the simulation.

