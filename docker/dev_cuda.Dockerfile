# FROM nvidia/cuda:12.2.2-base-ubuntu20.04
# FROM nvidia/cuda:12.0.0-base-ubuntu20.04
FROM nvidia/cuda:12.0.0-devel-ubuntu20.04

# This docker is intended to run on a development machine.
# No CUDA (for now), but with simulation and without sensor drivers.

# Based heavily on Piloting docker from Julian Keller

# Install ROS first. This should be the only image where you have to
# manually install ROS so...
# Taken from OSRF docker file.

RUN apt-get -qq update && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata 

# We're in Zurich!
ENV TZ="Europe/Zurich"

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full

# Copy scripts folder
COPY scripts/ /root/scripts/
WORKDIR /root/
RUN chmod a+x -R /root/scripts

# Env variables
ENV MOMA_DEP_WS=/root/moma_dep_ws
ENV SCRIPTS_PATH=/root/scripts

# Run the general dep installation
RUN scripts/install_sys_deps.sh

# Run the ROS workspace set-up and dep installation
RUN scripts/install_ros_deps.sh

# Run the driver (franka, RealSense, etc...) installation
RUN scripts/install_drivers.sh

# Run the gazebo simulation installation
RUN scripts/install_simulation.sh

# Install nikilesh's stuff
# RUN rm /root/scripts/requirements_nikhilesh.txt
# COPY scripts/requirements_nikhilesh.txt /root/scripts/requirements_nikhilesh.txt
# RUN scripts/install_nikhilesh.sh

# Finally, build all the stuff we downloaded.
# RUN scripts/build_ros.sh

# Install deps for the controller
# RUN apt update && apt install -y wget git
# RUN cd $MOMA_DEP_WS/ && wget https://raw.githubusercontent.com/matthias-mayr/Cartesian-Impedance-Controller/refs/heads/master/scripts/install_dependencies.sh
# RUN cd $MOMA_DEP_WS/ && chmod +x install_dependencies.sh
# RUN cd $MOMA_DEP_WS/ && ./install_dependencies.sh

# Install ARCHE installs
RUN scripts/install_arche25.sh

# Install Open3D pinned to version 0.19.0
RUN pip install open3d==0.19.0

# Install ROS package with caching-friendly apt usage
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-grid-map \
 && rm -rf /var/lib/apt/lists/*

# Finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh
