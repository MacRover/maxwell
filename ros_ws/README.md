# Maxwell Workspace

Welcome to the Maxwell WS! All of our custom ROS2 packages are located here.

For more info on ROS2 (Humble), click [here](https://docs.ros.org/en/humble/index.html).

### Build
Ensure you are in the `ros_ws` directory,
```bash
cd /path/to/maxwell/ros_ws
```
Install all dependencies:
```bash
# Only need to call once
# ---------------------------------------------------
sudo apt update && sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
# ---------------------------------------------------
rosdep install --from-paths src --ignore-src -y
```
Build and Source ROS packages:
```bash
colcon build --symlink-install
source install/setup.bash
```

### If Something Goes Wrong
Clean the WS and rebuild:
```bash
rm -rf build/ install/ log/ && colcon build --symlink-install
```

