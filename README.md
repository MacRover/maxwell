# Maxwell

## ROS Quick Start
1. Install ROS 2 Iron for Ubuntu 22.04 ([link to official guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html))
2. Configure `rosdep`
    ```
    sudo apt update && sudo apt install python3-rosdep -y
    sudo rosdep init
    rosdep update
    ```
3. Clone this repo
   ```
   git clone https://github.com/MacRover/maxwell.git
   ```
4. Use `rosdep` to install repo dependencies
   ```
   cd maxwell/ros_ws
   rosdep install --from-paths src --ignore-src -y
   ```
5. Build robot packages
   ```
   colcon build
   ```
