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

  sudo apt-get -qq install robotpkg-octomap=1.9.6 robotpkg-hpp-fcl=1.7.5 || fail "Error installing robotpkg libraries"

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

  mkdir -p ~/git
  git clone git@github.com:stack-of-tasks/pinocchio.git ~/git/pinocchio
  cd ~/git/pinocchio || fail
  git checkout v2.6.4
  git submodule update --init --recursive
  
  # Remove install dir if already exists
  if [ -d install ]; then rm -Rf install; fi
  rm -r install
  mkdir install
  PINOCCHIO_INSTALL_PREFIX=${HOME}/git/pinocchio/install
  PINOCCHIO_INSTALL_PREFIX_STR=\${HOME}/git/pinocchio/install


  # Remove build dir if it already exists
  if [ -d build ]; then rm -Rf build; fi
  mkdir build
  cd build
  
  # Fails if run for the first time, because hpp-fcl is not found. A resource of the bashrc fixes it.
  cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${PINOCCHIO_INSTALL_PREFIX} -DBUILD_WITH_COLLISION_SUPPORT=ON || fail "Please resource ~/.moma_bashrc and restart the script"
  make -j4 || fail "Error building pinocchio"
  sudo make install || fail "Error installing pinocchio"

  cat << EOF >> ~/.moma_bashrc
export PATH=${PINOCCHIO_INSTALL_PREFIX}/bin:\$PATH
export PKG_CONFIG_PATH=${PINOCCHIO_INSTALL_PREFIX_STR}/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=${PINOCCHIO_INSTALL_PREFIX_STR}/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=\$PYTHONPATH:${PINOCCHIO_INSTALL_PREFIX_STR}/lib/python2.7/dist-packages
export CMAKE_PREFIX_PATH=${PINOCCHIO_INSTALL_PREFIX_STR}:\$CMAKE_PREFIX_PATH
EOF
}


install_control() {
  ROBOTPKG_NAMES=("robotpkg-octomap" "robotpkg-hpp-fcl")
  dpkg -s "${ROBOTPKG_NAMES[@]}" >/dev/null 2>&1 || install_robotpkg
  [ -d "${HOME}/git/pinocchio/install/share/pinocchio" ] || install_pinocchio
  info "Control dependencies installation successful"
}

install_system_deps() {
sudo apt-get install \
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
	qtbase5-dev -y || fail "Error installing system dependencies"
}


install_external() {
	wstool merge -t . ./moma/moma_ssh.rosinstall || fail "Error merging rosinstall"
	wstool update || fail "Error updating wstool. Installing external dependencies failed"
}

usage="$(basename "$0") [-h] [-c --control] -- moma stack installation script\n
where:\n
    -h  show this help text\n
    -c  install control dependencies (required to build custom controllers)\n"

POSITIONAL=()
INSTALL_CONTROL_DEPS=false
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
    *)    # unknown option
      POSITIONAL+=("$1") # save it in an array for later
      shift # past argument
      ;;
  esac
done

set -- "${POSITIONAL[@]}" # restore positional parameters

info "Installing control dependencies  = ${INSTALL_CONTROL_DEPS}"


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
if [ $INSTALL_CONTROL_DEPS ]
then
  info "Installing control dependencies"
  install_control
fi

info "Sourcing moma workspace"
echo "source ${CATKIN_WS}/devel/setub.bash" >> ~/.moma_bashrc
source ~/.moma_bashrc

catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo || fail
info "Installation complete" 
info "Now you can build moma packages"
