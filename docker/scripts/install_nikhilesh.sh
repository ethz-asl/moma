#!/bin/bash
set -e  # Exit on error
set -o pipefail  # Ensure pipeline failures are propagated
export DEBIAN_FRONTEND=noninteractive  # Suppress interactive prompts

echo "Updating system packages..."
# apt-get -qq update && apt-get -qq upgrade -y

# BASICS
echo "Installing basic dependencies..."
apt-get update && apt-get install -y nano vim 
echo "Installing basic dependencies...done"

# ROS
# Install Kalibr stuff
# apt-get -qq update && apt-get install -y git wget autoconf automake nano \
#     libeigen3-dev libboost-all-dev libsuitesparse-dev \
#     doxygen libopencv-dev \
#     libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev

# For voxblox:
# apt-get -qq update && apt-get install ros-noetic-grpc build-essential libtool
# apt-get install -y python3-dev python3-pip python3-scipy \
#     python3-matplotlib ipython3 python3-igraph python3-pyx python3-tk
# apt-get install -y python3-wxgtk4.0

# this is for nvblox
# pip install --upgrade cmake
# apt-get -qq update &&  apt-get install -y libgoogle-glog-dev libgtest-dev libgflags-dev python3-dev libsqlite3-dev

# DRIVER
# Bota
echo "Installing Bota driver..."
apt-get update -qq && apt install -y ros-noetic-bota-driver
echo "Installing Bota driver...done"

# PYTHON
echo "Installing Python dependencies from requirements_nikhilesh.txt..."
pip3 install --upgrade pip
echo "Installing pip dependencies..."
pip3 install --no-cache-dir -r "$SCRIPTS_PATH/requirements_nikhilesh.txt"
echo "Installing Python dependencies from requirements_nikhilesh.txt...done"

# Clear cache to reduce Docker image size
echo "Cleaning up unnecessary files..."
rm -rf /var/lib/apt/lists/* ~/.cache/pip
