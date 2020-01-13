FROM ros:kinetic-ros-base

RUN apt-get update && apt-get install -y git

ENV ROS_DISTRO kinetic

ENV ROS_WORKSPACE /home/ros_workspace
ENV ROS_WORKSPACE_SRC /home/ros_workspace/src

RUN mkdir -p $ROS_WORKSPACE_SRC

ENV SOURCE_WS source /opt/ros/$ROS_DISTRO/setup.bash

RUN cd $ROS_WORKSPACE_SRC && /bin/bash -c "$SOURCE_WS && catkin_init_workspace"
RUN cd $ROS_WORKSPACE && /bin/bash -c "$SOURCE_WS && catkin_make"

# ========= Ridgeback Setup ===============

# Install Ridgeback ros packages.
RUN apt-get install -y ros-$ROS_DISTRO-amcl \
                        ros-$ROS_DISTRO-roslint \
                        ros-$ROS_DISTRO-gmapping \
                        ros-$ROS_DISTRO-interactive-marker-twist-server \
                        ros-$ROS_DISTRO-lms1xx \
                        ros-$ROS_DISTRO-map-server \
                        ros-$ROS_DISTRO-puma-motor-driver \
                        ros-$ROS_DISTRO-robot-localization \
                        ros-$ROS_DISTRO-rosserial \
                        ros-$ROS_DISTRO-rosserial-server \
                        ros-$ROS_DISTRO-serial \
                        ros-$ROS_DISTRO-teleop-twist-joy \
                        ros-$ROS_DISTRO-urdf \
                        ros-$ROS_DISTRO-ros-control \
                        ros-$ROS_DISTRO-ros-controllers \
                        ros-$ROS_DISTRO-laser-geometry \
                        ros-$ROS_DISTRO-pcl-ros \
                        ros-$ROS_DISTRO-xacro \
                        ros-$ROS_DISTRO-robot-state-publisher \
                        ros-$ROS_DISTRO-imu-filter-madgwick \
                        ros-$ROS_DISTRO-diagnostic-aggregator \
                        ros-$ROS_DISTRO-urg-node \
                        ros-$ROS_DISTRO-navigation \
                        socat \
                        less \
                        iputils-ping

RUN git clone https://github.com/ridgeback/ridgeback_robot $ROS_WORKSPACE_SRC/ridgeback_robot
RUN git clone https://github.com/ridgeback/ridgeback $ROS_WORKSPACE_SRC/ridgeback
RUN git clone https://github.com/iralabdisco/ira_laser_tools $ROS_WORKSPACE_SRC/ira_laser_tools

# Overwrite some of the ridgeback configurations.
COPY launch/accessories.launch $ROS_WORKSPACE_SRC/ridgeback_robot/ridgeback_bringup/launch/accessories.launch

RUN cd $ROS_WORKSPACE && /bin/bash -c "$SOURCE_WS && catkin_make -DCMAKE_BUILD_TYPE=Release"

RUN mkdir -p /home/launch
COPY launch/* /home/launch/

# =============== OTHER STUFF =======================================

# Place config files
RUN mkdir -p /home/config
COPY config/ros_setup.bash /home/config/

# ======== Entrypoint =============
COPY ./asl_entrypoint.bash /
ENTRYPOINT ["/asl_entrypoint.bash"]
CMD /opt/ros/$ROS_DISTRO/bin/roslaunch /home/launch/ros.launch

