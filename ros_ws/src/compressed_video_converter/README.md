# Compressed Video Converter Documentation 

Below I am just going to go through my steps and things I did while setting this up for both my and your references.

## Setup Foxglove
1. Running foxglove bridg on the jetson:
```bash
ros2 run foxglove_bridge foxglove_bridge
```
- if that isn't installed make sure to install: 
```bash
sudo apt update
sudo apt install ros-humble-foxglove-bridge
```

2. On your local machine open up your foxglove and in the foxglove websocket put in:
```ws://mmrt@bigbrain.marsatmac.ca:8765```


## Running the usb cam:
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/mmrt/maxwell/ros_ws/src/usb_cam_config/config/params1.yaml

```
Notes:
- The port may need to be changed i.e dev/video2 (that's defined in the params file)
- If you're having issues with the video type it's because of this issue with the pixel format, if you wanted to fix it without a params file you'd do this:
1. Go to the path under the docker container to modify the data format:
2. cd /opt/ros/humble/share/usb_cam/config/
3. sudo nano params_1.yaml
4. Then go to the line that says "Pixel Format" and change mjpeg2rgb → yuyv2rgb


## Launching the docker container:
okay to run the container on maxwell go into the directory: 
```bash
cd ~/Documents/isaac_ros-dev/src/isaac_ros_common
```
and run:
```bash:
./scripts/run_dev.sh 
```
This link goes over the setup including running the docker container at one point:
https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_compression/isaac_ros_h264_encoder/index.html#quickstart

What seemed to be the steps inside the container was:
```bash 
source install/setup.bash
```
Then run:
ros2 launch isaac_ros_h264_encoder isaac_ros_h264_encoder.launch.py



Very important Note, for some reason newer versions of setuptools are giving me a headache with this node, so please use:
```bash
pip install "setuptools==65.5.1"
```


Running the node: 
```bash
colcon build --packages-select h264_decoder
source install/setup.bash
ros2 run h264_decoder h264_decoder_node 
```
