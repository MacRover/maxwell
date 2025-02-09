# Angle Publisher

This package is a ROS2 node that publishes angle data for actuated camera .

## Launch File

To run the launch file for the angle publisher node, use the following command:

```sh
ros2 launch angle_publisher servo_controller.launch
```

## Changing the Joy Rate

The joy rate can be changed by modifying the `joy_rate` parameter in the launch file. Open the `servo_controller.launch` file and set the `joy_rate` parameter to your desired value:

<param name="joy_rate" type="double" value="10.0" />

