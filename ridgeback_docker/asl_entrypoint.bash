#!/bin/bash

# Source the ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS_WORKSPACE/devel/setup.bash

# Set hostname
echo "127.0.0.1 asl-ridgeback" >> /etc/hosts

# Set custom environment variables
source /home/config/ros_setup.bash

# Execute any commands that might be defined by CMD or passed to docker run
exec "$@"
