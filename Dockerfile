FROM ros:noetic-robot

# disable interactive prompt
ENV DEBIAN_FRONTEND=noninteractive

# update repos
RUN apt-get update && apt-get -qq upgrade

# update git and set to always point to https
RUN apt-get install -y git
RUN git config --global url.https://github.com/.insteadOf git@github.com:
RUN apt install curl

# get install tools
RUN echo "Hello "
RUN apt-get install -y python3-catkin-tools python3-vcstool

# create a catkin workspace
RUN mkdir -p ${HOME}/catkin_ws/src/moma
ENV CATKIN_WS=${HOME}/catkin_ws
WORKDIR ${CATKIN_WS}
RUN catkin init
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY install_dependencies.sh *.repos ${CATKIN_WS}/src/moma/
WORKDIR ${CATKIN_WS}/src
RUN vcs import --recursive --input moma/moma_core.repos
RUN vcs import --recursive --input moma/moma_piloting.repos
RUN vcs import --input https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
RUN DEBIAN_FRONTEND=noninteractive moma/install_dependencies.sh --control
RUN rm -r ${CATKIN_WS}/src/moma