#!/bin/bash
export DEBIAN_FRONTEND=noninteractive

# source ROS
source /opt/ros/$ROS_DISTRO/setup.bash

#DISTRIB_RELEASE=$(lsb_release -sr)
. /etc/lsb-release

RED=`tput setaf 1`
GREEN=`tput setaf 2`
NC=`tput sgr0`

info() {
  echo ${GREEN}$1${NC}
}

fail() {
    echo ${RED}[STOPPING DUE TO ERROR] $1${NC} >&2
    exit 1
}

install_robotpkg() {
  echo "Installing robotpkg libraries"

  sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $DISTRIB_CODENAME robotpkg
EOF

  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |
    sudo apt-key add -

  sudo apt-get -qq update

  sudo apt-get -qq install robotpkg-octomap=1.9.6 robotpkg-hpp-fcl=1.7.8 || fail "Error installing robotpkg libraries"

  cat << EOF >> ~/.moma_bashrc
export PATH=/opt/openrobots/bin:\$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH
EOF
}

install_pinocchio() {
  echo "Installing pinocchio"
  source ~/.moma_bashrc

  mkdir -p ~/git
  git clone git@github.com:stack-of-tasks/pinocchio.git ~/git/pinocchio
  cd ~/git/pinocchio || fail "Failed to clone pinocchio repo"
  git checkout v2.6.4
  git submodule update --init --recursive

  PINOCCHIO_INSTALL_PREFIX=${HOME}/git/pinocchio/install
  PINOCCHIO_INSTALL_PREFIX_STR=\${HOME}/git/pinocchio/install

  # Fails if run for the first time, because hpp-fcl is not found. A resource of the bashrc fixes it.
  # If no previous install is found build the package again
  if [[ ! -d install ]]
  then
      [ ! -d build ] || rm -r build
      mkdir build
      cd build

      cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${PINOCCHIO_INSTALL_PREFIX} -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_PYTHON_INTERFACE=ON -DBUILD_TESTING=OFF || fail "Please resource ~/.moma_bashrc and restart the script"
      make -j4 || fail "Error building pinocchio"

      mkdir install
      make install || fail "Error installing pinocchio"
  else
    info "Previos pinocchio installation found at ${PINOCCHIO_INSTALL_PREFIX}"
  fi


  cat << EOF >> ~/.moma_bashrc
export PATH=${PINOCCHIO_INSTALL_PREFIX}/bin:\$PATH
export PKG_CONFIG_PATH=${PINOCCHIO_INSTALL_PREFIX_STR}/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=${PINOCCHIO_INSTALL_PREFIX_STR}/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=\$PYTHONPATH:${PINOCCHIO_INSTALL_PREFIX_STR}/lib/python2.7/dist-packages:${PINOCCHIO_INSTALL_PREFIX_STR}/lib/python3/dist-packages
export CMAKE_PREFIX_PATH=${PINOCCHIO_INSTALL_PREFIX_STR}:\$CMAKE_PREFIX_PATH
EOF
}


install_mavsdk() {
  mkdir -p ~/git
  git clone https://github.com/fada-catec/piloting-mavsdk ~/git/piloting-mavsdk
  cd ~/git/piloting-mavsdk
  mkdir install
  mkdir build && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=${HOME}/git/piloting-mavsdk/install
  make -j4
  make install

  cat << EOF >> ~/.moma_bashrc
export LD_LIBRARY_PATH=${HOME}/git/piloting-mavsdk/install/lib:\$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=${HOME}/git/piloting-mavsdk/install:\$CMAKE_PREFIX_PATH
EOF
}


install_control() {
  #ROBOTPKG_NAMES=("robotpkg-octomap" "robotpkg-hpp-fcl")
  #dpkg -s "${ROBOTPKG_NAMES[@]}" >/dev/null 2>&1 || install_robotpkg
  install_robotpkg
  install_pinocchio
  info "Control dependencies installation successful"
}

install_piloting() {
  install_mavsdk
  info "Piloting dependencies installation successful"
}

install_system_deps() {
sudo --preserve-env=DEBIAN_FRONTEND apt-get install \
	ros-$ROS_DISTRO-ros-control \
	ros-$ROS_DISTRO-ros-controllers \
	ros-$ROS_DISTRO-gazebo-ros-pkgs \
	ros-$ROS_DISTRO-gazebo-ros-control \
	ros-$ROS_DISTRO-gazebo-ros \
	ros-$ROS_DISTRO-moveit \
	ros-$ROS_DISTRO-rosmon \
	ros-$ROS_DISTRO-pcl-ros \
	ros-$ROS_DISTRO-tf2-sensor-msgs \
	ros-$ROS_DISTRO-py-trees \
	ros-$ROS_DISTRO-py-trees-ros \
	ros-$ROS_DISTRO-rqt-py-trees \
	ros-$ROS_DISTRO-libfranka \
	ros-$ROS_DISTRO-joint-state-publisher-gui \
	ros-$ROS_DISTRO-ddynamic-reconfigure \
	ros-$ROS_DISTRO-lms1xx \
	ros-$ROS_DISTRO-interactive-marker-twist-server \
  ros-$ROS_DISTRO-plotjuggler-ros \
  ros-$ROS_DISTRO-jsk-rviz-plugins \
	qtbase5-dev -qq || fail "Error installing system dependencies"
}


install_external() {
	vcs import --recursive --input moma/moma_core.repos || fail "Error importing dependencies"
}

usage="$(basename "$0") [-h] [-c --control] [-p --piloting] -- moma stack installation script\n
where:\n
    -h  show this help text\n
    -c  install control dependencies (required to build custom controllers)\n
    -p  install piloting dependencies (required to build piloting demo)\n"

POSITIONAL=()
INSTALL_CONTROL_DEPS=false
INSTALL_PILOTING_DEPS=false
while [[ $# -gt 0 ]]; do
  key="$1"

  case $key in
    -h|--help)
      echo -e $usage
      exit 0
      ;;
    -c|--control)
      INSTALL_CONTROL_DEPS=true
      shift # past argument
      ;;
    -p|--piloting)
      INSTALL_PILOTING_DEPS=true
      shift # past argument
      ;;
    *)    # unknown option
      POSITIONAL+=("$1") # save it in an array for later
      shift # past argument
      ;;
  esac
done

set -- "${POSITIONAL[@]}" # restore positional parameters

info "Installing control dependencies  = ${INSTALL_CONTROL_DEPS}"
info "Installing piloting dependencies  = ${INSTALL_PILOTING_DEPS}"


if [ "$DISTRIB_RELEASE" == "20.04" ] && [ "$ROS_DISTRO" == "noetic" ]; then
  echo "${GREEN}Supported distribution was found${NC}"
else
  fail "Your distribution is currently not officially supported"
fi

[ -n "$CATKIN_WS" ] || fail "Please set the CATKIN_WS variable first"

echo "Starting installation"
[ ! -e ~/.moma_bashrc ] || rm ~/.moma_bashrc
touch ~/.moma_bashrc


install_system_deps
install_external
if $INSTALL_CONTROL_DEPS
then
  info "Installing control dependencies"
  install_control
fi
if $INSTALL_PILOTING_DEPS
then
  info "Installing piloting dependencies"
  install_piloting
fi

info "Sourcing moma workspace"
echo "source ${CATKIN_WS}/devel/setup.bash || true" >> ~/.moma_bashrc
source ~/.moma_bashrc

cd ${CATKIN_WS}
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo || fail
info "Installation complete"
info "Now you can build moma packages"
