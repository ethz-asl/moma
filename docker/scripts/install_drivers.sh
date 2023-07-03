#!/bin/bash

set -o pipefail

# Install RealSense
# Directions from: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
apt-get update -qq && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# Install Franka stuff
apt-get -qq update && apt-get install -y ros-noetic-franka-ros 

# Install the source code in ROS
cd $MOMA_DEP_WS/src
vcs import --recursive --input $SCRIPTS_PATH/moma_drivers.repos
