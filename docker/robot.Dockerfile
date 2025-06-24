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

# Install the new impedance controller
RUN git clone https://github.com/matthias-mayr/Cartesian-Impedance-Controller.git
RUN mv Cartesian-Impedance-Controller $MOMA_DEP_WS/src
RUN cd $MOMA_DEP_WS && ./src/Cartesian-Impedance-Controller/scripts/install_dependencies.sh

# Finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh
