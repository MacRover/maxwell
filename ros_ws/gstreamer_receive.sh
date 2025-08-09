gst-launch-1.0 udpsrc port=40627 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream !   avdec_h265 !   videoconvert ! autovideosink sync=false &

sleep 1

gst-launch-1.0 udpsrc port=40628 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream !   avdec_h265 !   videoconvert ! autovideosink sync=false &

sleep 1

gst-launch-1.0 udpsrc port=40629 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream !   avdec_h265 !   videoconvert ! autovideosink sync=false &

sleep 1

gst-launch-1.0 udpsrc port=40630 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream !   avdec_h265 !   videoconvert ! autovideosink sync=false &

sleep 1

gst-launch-1.0 udpsrc port=40631 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream !   avdec_h265 !   videoconvert ! autovideosink sync=false &

sleep 1

echo "Started Camera Receive"

wait

# exit with control c

# Experimental Pipelines

 

#gst-launch-1.0 udpsrc port=40627 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream leaky=downstream !   avdec_h265 !   videoconvert !  autovideosink sync=false &


#Use VAAPI decode - sudo apt-get install gstreamer1.0-vaapi??
#gst-launch-1.0 udpsrc port=40627 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream !   vaapih265dec !   videoconvert !  autovideosink sync=false &

## run 'htop' to see cpu usage during decoding