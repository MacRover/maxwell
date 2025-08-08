#!/bin/sh
# flashlight.sh

# enter first argument as "on" or "off"

if [ "$1" = "on" ]; then
    ros2 service call /flashlight_control std_srvs/srv/SetBool "{data: true}"
    echo "Flashlight requested ON"
elif [ "$1" = "off" ]; then
    ros2 service call /flashlight_control std_srvs/srv/SetBool "{data: false}"
    echo "Flashlight requested OFF"
else
    echo "Usage: ./flashlight.bash [on|off]"
    exit 1
fi


