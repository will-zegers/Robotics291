#!/bin/bash

if [ $# -ne 1 ]; then
    echo "usage: $0: {start|stop}"
    exit 1
fi


case "$1" in
    start)
        export ROS_MASTER_URI=http://192.168.100.1:11311
        rosrun image_view image_view image:=/track_opencv_result_left &
        rosrun image_view image_view image:=/track_opencv_result_right &
        jobs
        ;;
     
    stop)
        killall image_view
        ;;
     
    *)
        echo $"Usage: $0 {start|stop}"
        exit 1
esac