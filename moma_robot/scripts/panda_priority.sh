#!/bin/bash

until pid=$(pgrep franka_control)
do
    sleep 1
    echo "Panda control priority could not be set yet, retrying..."
done

sudo renice -20 -p $pid

exit 0
