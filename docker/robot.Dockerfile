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

# Install Open3D pinned to version 0.19.0
RUN pip install open3d==0.19.0

# Install ROS package with caching-friendly apt usage
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-grid-map nano vim usbutils \
 && rm -rf /var/lib/apt/lists/*

# Finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh

# Just for the debug heightmap (model_eval_utils module-level import).
# 2.0.7 is the last release with a cp38 wheel; 2.1.x needs Python >=3.10.
RUN pip install shapely==2.0.7