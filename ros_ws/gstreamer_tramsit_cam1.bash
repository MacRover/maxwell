#!/bin/bash

# run in loop in case pipeline crashes
while true
do
echo "Starting camera stream"
#timeout 30s
gst-launch-1.0 v4l2src device=/dev/video6 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 !   h265parse !   rtph265pay config-interval=1 mtu=1500 aggregate-mode=1 !   udpsink host=192.168.1.133 port=40627 sync=false max-lateness=333333
sleep 1
done


## Experimental pipleine transmit!
#https://gist.github.com/sandman/83bc66cf33bd67ec5476d823d37381df 

# Confirm if camera is working well: capsfilter caps="video/x-raw,format=NV12,width=640,height=480,framerate=25/1"

#gst-launch-1.0 v4l2src device=/dev/video0 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 !   h265parse !   rtph265pay config-interval=1 mtu=1400 aggregate-mode=1 !   udpsink host=192.168.1.199 port=40627 sync=false max-lateness=33333333

#enable max performance, but may use a lot more power

#gst-launch-1.0 v4l2src device=/dev/video0 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 maxperf-enable=1 !   h265parse !   rtph265pay config-interval=1 mtu=1400 aggregate-mode=1 !   udpsink host=192.168.1.199 port=40627 sync=false max-lateness=33333333


# VIDIOC_ENUM_FMT
#         Type: Video Capture

#         [0]: 'YUYV' (YUYV 4:2:2)
#                 Size: Discrete 424x240
#                         Interval: Discrete 0.017s (60.000 fps)
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.167s (6.000 fps)
#                 Size: Discrete 640x480
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.167s (6.000 fps)
#                 Size: Discrete 1280x720
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.167s (6.000 fps)
#                 Size: Discrete 1920x1080
#                         Interval: Discrete 0.125s (8.000 fps)
