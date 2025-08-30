FROM osrf/ros:noetic-desktop-full

# This docker is intended to run on the robot and controlling the arm.
# No CUDA but with sensor drivers.

# Based heavily on Piloting docker from Julian Keller

# Copy scripts folder
COPY scripts/ /root/scripts/
WORKDIR /root/
RUN chmod a+x -R /root/scripts

# Env variables
ENV MOMA_DEP_WS=/root/moma_dep_ws
ENV MOMA_WS=/root/moma_ws
ENV SCRIPTS_PATH=/root/scripts

# Run the general dep installation
RUN scripts/install_sys_deps.sh

# Run the ROS workspace set-up and dep installation
RUN scripts/install_ros_deps.sh

# Run the driver (franka, RealSense, etc...) installation
RUN scripts/install_drivers.sh

# Run the gazebo simulation installation
RUN scripts/install_simulation.sh

# Install ARCHE installs
RUN scripts/install_arche25.sh

# Add these here to avoid rebuilding everything if only these change. Move into install script later.
RUN pip install open3d
RUN apt update && apt install ros-noetic-grid-map

# Finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh
