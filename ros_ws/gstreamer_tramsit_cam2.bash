#!/bin/bash

# run in loop in case pipeline crashes
while true
do
echo "Starting camera stream"
#timeout 30s
gst-launch-1.0 v4l2src device=/dev/video10 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=24/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 !   h265parse !   rtph265pay config-interval=1 mtu=1500 aggregate-mode=1 !   udpsink host=192.168.1.199 port=40630 sync=false max-lateness=333333
sleep 1
done


## Experimental pipleine transmit!
#https://gist.github.com/sandman/83bc66cf33bd67ec5476d823d37381df 

# Confirm if camera is working well: capsfilter caps="video/x-raw,format=NV12,width=640,height=480,framerate=25/1"

#gst-launch-1.0 v4l2src device=/dev/video0 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 !   h265parse !   rtph265pay config-interval=1 mtu=1400 aggregate-mode=1 !   udpsink host=192.168.1.199 port=40627 sync=false max-lateness=33333333

#enable max performance, but may use a lot more power

#gst-launch-1.0 v4l2src device=/dev/video0 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 maxperf-enable=1 !   h265parse !   rtph265pay config-interval=1 mtu=1400 aggregate-mode=1 !   udpsink host=192.168.1.199 port=40627 sync=false max-lateness=33333333

# [0]: 'YUYV' (YUYV 4:2:2)
#                 Size: Discrete 640x480
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 160x120
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 176x144
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 320x176
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 320x240
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 432x240
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 352x288
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 544x288
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 640x360
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 752x416
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 800x448
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 864x480
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 960x544
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1024x576
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 800x600
#                         Interval: Discrete 0.042s (24.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1184x656
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 960x720
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1280x720
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1392x768
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1504x832
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1600x896
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1280x960
#                         Interval: Discrete 0.133s (7.500 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1712x960
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1792x1008
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1920x1080
#                         Interval: Discrete 0.200s (5.000 fps)