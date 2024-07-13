Use rosdep to install moveit and dependencies, or using `apt install install ros-iron-moveit`. Can also be installed from source using [instructions](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

Launch demo using:

```
ros2 launch arm_moveit_config demo.launch.py
```

TODO:

- Hardware data needs to be published to `/joint_states` topic which uses [JointState](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html) message. Joint names can be found from `ros2_controllers.yaml` file. 
