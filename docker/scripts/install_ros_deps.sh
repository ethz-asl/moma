#!/bin/bash
set -o pipefail

# Set up a ROS workspace
mkdir -p $MOMA_DEP_WS/src
cd $MOMA_DEP_WS
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install ROS packages from apt
apt-get install -y \
	ros-noetic-ros-control \
	ros-noetic-ros-controllers \
	ros-noetic-moveit \
	ros-noetic-rosmon \
	ros-noetic-pcl-ros \
	ros-noetic-tf2-sensor-msgs \
	ros-noetic-py-trees \
	ros-noetic-py-trees-ros \
	ros-noetic-rqt-py-trees \
	ros-noetic-joint-state-publisher-gui \
	ros-noetic-ddynamic-reconfigure \
	ros-noetic-interactive-marker-twist-server \
	ros-noetic-ros-numpy \
	ros-noetic-smach \
	ros-noetic-smach-ros \
	ros-noetic-tf-conversions \
	ros-noetic-rviz-visual-tools \
  ros-noetic-fkie-multimaster \
  ros-noetic-fkie-node-manager

# Install all the other dependencies in the moma_dep_ws
cd $MOMA_DEP_WS/src || exit 1
vcs import --recursive --input $SCRIPTS_PATH/moma_ros_deps.repos

