#!/bin/bash

# run in loop in case pipeline crashes
while true
do
echo "Starting camera stream"
#timeout 30s
gst-launch-1.0 v4l2src device=/dev/video0 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.199 port=40627 sync=false
sleep 1
done


## Experimental pipleine transmit!
#https://gist.github.com/sandman/83bc66cf33bd67ec5476d823d37381df 

# Confirm if camera is working well: capsfilter caps="video/x-raw,format=NV12,width=640,height=480,framerate=25/1"

#gst-launch-1.0 v4l2src device=/dev/video0 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 !   h265parse !   rtph265pay config-interval=1 mtu=1400 aggregate-mode=1 !   udpsink host=192.168.1.199 port=40627 sync=false max-lateness=33333333

#enable max performance, but may use a lot more power

#gst-launch-1.0 v4l2src device=/dev/video0 io-mode=2 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc bitrate=2000000 preset-level=1 control-rate=1 maxperf-enable=1 !   h265parse !   rtph265pay config-interval=1 mtu=1400 aggregate-mode=1 !   udpsink host=192.168.1.199 port=40627 sync=false max-lateness=33333333
