FROM ros:noetic-robot AS deps

# disable interactive prompt
ENV DEBIAN_FRONTEND=noninteractive

# update repos
RUN apt-get -qq update && apt-get -qq upgrade

# update git and set to always point to https
RUN apt-get -qq update && apt-get install -y curl
RUN apt-get -qq update && apt-get install -y git git-lfs
RUN git config --global url.https://github.com/.insteadOf git@github.com:

# get install tools
RUN apt-get -qq update && apt-get install -y python3-catkin-tools python3-vcstool python3-pip

# create a catkin workspace
RUN mkdir -p /root/catkin_ws/src/moma
ENV CATKIN_WS=/root/catkin_ws
WORKDIR ${CATKIN_WS}
RUN catkin init

COPY install_dependencies.sh *.repos ${CATKIN_WS}/src/moma/
COPY moma_mission/requirements.txt ${CATKIN_WS}/src/moma/moma_mission/
WORKDIR ${CATKIN_WS}/src
RUN vcs import --recursive --input moma/moma_core.repos
RUN vcs import --recursive --input moma/moma_piloting.repos
# Fix recent versions of Ubuntu putting /opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig into PKG_CONFIG_PATH
# and then pinocchio will have a non-existent include path when queried with "pkg-config --cflags pinocchio"
ENV PKG_CONFIG_PATH=""
RUN apt-get -qq update && apt-get -qq upgrade && DEBIAN_FRONTEND=noninteractive moma/install_dependencies.sh --control --piloting
RUN rosdep update
RUN rosdep install --from-paths . --ignore-src -r -y || true

FROM deps AS build
COPY . ${CATKIN_WS}/src/moma/
COPY ros_entrypoint.sh /
WORKDIR ${CATKIN_WS}
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
RUN catkin config --extend /opt/ros/noetic
SHELL ["/bin/bash", "-c"]
RUN source ~/.moma_bashrc && catkin build piloting_demo
# Redundant
ENTRYPOINT ["/ros_entrypoint.sh"]

FROM piloting/noetic-release:latest AS build-cached
COPY . ${CATKIN_WS}/src/moma/
RUN source ~/.moma_bashrc && catkin build piloting_demo
