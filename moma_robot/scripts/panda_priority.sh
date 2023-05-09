#!/bin/bash

while true
do
    until pid=$(pgrep franka_control)
    do
        sleep 1
        echo "Panda control priority could not be set yet, retrying..."
    done

    if ! ps -o ni $pid | grep -q '\-20'; then
        sudo renice -20 -p $pid
    fi
    sleep 1
done

exit 0
