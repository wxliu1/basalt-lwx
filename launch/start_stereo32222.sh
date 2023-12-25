#!/bin/bash

BASEDIR='/root/dev/stereo3_ros1_ws/install/lib/stereo3'

while true; do

    PRO_NOW=`pgrep -f "stereo3" | wc -l 2>/dev/null`

    if [ $PRO_NOW -eq 0 ]; then
        echo " reboot stereo3... "
        cd $BASEDIR
        ./stereo31  &
    else
        echo "`date`  stereo3 is running..."
    fi
    sleep 3

done
