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

## Setup for the warehouse.sdf world:
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress

## TODO

- Adding more controls in the keyboard controller
- Figuring out the model, how will a odom message be converted to controller commands