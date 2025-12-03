FROM osrf/ros:noetic-desktop-full

# This docker is intended to run on a development machine.
# No CUDA (for now), but with simulation and without sensor drivers.

# Based heavily on Piloting docker from Julian Keller

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

# Install realsense and friends
RUN scripts/install_drivers.sh

# Run the gazebo simulation installation
RUN scripts/install_simulation.sh

# Install python packages
RUN scripts/install_python_packages.sh

# Finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh
