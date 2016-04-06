#!/bin/bash

if [ $# -ne 1 ]; then
    echo "usage: $0: {start|stop}"
    exit 1
fi


case "$1" in
    start)
        ### KILL EVERYTHING
        killall ball_node
        killall roscore
        killall roslaunch

        ### START
        echo -e "\n\n########## STARTING ROBOT "
        roscore &
        sleep 5
        roslaunch ball stereo.launch &
        sleep 5
        roslaunch ball ball.launch &
        sleep 5
        echo -e "\n\n########## ROBOT READY?? CHECK FOR ANY ERRORS "
        jobs
        ;;
     
    stop)
        ### KILL EVERYTHING
        killall ball_node
        killall roscore
        killall roslaunch
        ;;
     
    *)
        echo $"Usage: $0 {start|stop}"
        exit 1
esac