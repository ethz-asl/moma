FROM osrf/ros:noetic-desktop-full

# This docker is intended to run on the robot and controlling the arm.
# No CUDA but with sensor drivers.

# Based heavily on Piloting docker from Julian Keller

# update repos
RUN apt-get -qq update && apt-get -qq upgrade

# Copy scripts folder
COPY scripts/ /root/scripts/
WORKDIR /root/
RUN chmod a+x -R /root/scripts

# Run the general dep installation
RUN scripts/install_dependencies.sh

# Run the driver (franka, RealSense, etc...) installation
RUN scripts/install_drivers.sh





