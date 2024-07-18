#!/bin/bash

set -o pipefail

# Install RealSense
# Directions from: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
echo "deb https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key C8B3A55A6F3EFCDE

apt-get update -qq && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# Install Franka stuff
apt-get -qq update && apt-get install -y ros-noetic-franka-ros 

# Install Kalibr stuff
apt-get -qq update && apt-get install -y git wget autoconf automake nano \    
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    doxygen libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev libsuitesparse-dev

# apt-get install -y python3-dev python3-pip python3-scipy \
    # python3-matplotlib ipython3 python3-igraph python3-pyx python3-tk
# apt-get install -y python3-wxgtk4.0

# Install the source code in ROS
cd $MOMA_DEP_WS/src
vcs import --recursive --input $SCRIPTS_PATH/moma_drivers.repos

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
