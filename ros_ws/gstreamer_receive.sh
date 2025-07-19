gst-launch-1.0 udpsrc port=40627 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=500 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=15/1 !   autovideosink sync=false &

sleep 1

echo "Started Camera Receive"

wait

# exit with control c

# Experimental Pipelines

 

#gst-launch-1.0 udpsrc port=40627 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=500 max-size-time=0 max-size-bytes=0 leaky=downstream !   avdec_h265 !   videoconvert !  autovideosink sync=false &


#Use VAAPI decode - sudo apt-get install gstreamer1.0-vaapi??
#gst-launch-1.0 udpsrc port=40627 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=500 max-size-time=0 max-size-bytes=0 !   vaapih265dec !   videoconvert !  autovideosink sync=false &

## run 'htop' to see cpu usage during decoding