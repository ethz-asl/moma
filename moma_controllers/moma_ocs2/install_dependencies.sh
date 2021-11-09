#DISTRIB_RELEASE=$(lsb_release -sr)
. /etc/lsb-release

RED=`tput setaf 1`
GREEN=`tput setaf 2`
NC=`tput sgr0`

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

  sudo apt-get -qq install robotpkg-octomap=1.6.1 robotpkg-hpp-fcl=1.6.0 || fail "Error installing robotpkg libraries"

  cat << EOF >> ~/.bashrc
export PATH=/opt/openrobots/bin:\$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH
EOF
  . ~/.bashrc
}

install_pinocchio() {
  echo "Installing pinocchio"

  mkdir -p ~/Downloads
  git clone git@github.com:stack-of-tasks/pinocchio.git ~/Downloads/pinocchio
  cd ~/Downloads/pinocchio || fail
  git checkout 29be057af1beb
  git submodule update --init --recursive
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_WITH_COLLISION_SUPPORT=ON || fail "Please resource ~/.bashrc and restart the script"
  make -j4 || fail "Error building pinocchio"
  sudo make install || fail "Error installing pinocchio"

  cat << EOF >> ~/.bashrc
export PATH=/usr/local/bin:\$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=\$PYTHONPATH:/usr/local/lib/python2.7/dist-packages
export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH
EOF
  . ~/.bashrc
}

install_cmake() {
  # Hack for easily installing cmake >= 3.11
  sudo tee /etc/apt/sources.list.d/focal.list <<EOF
deb [trusted=yes] http://archive.ubuntu.com/ubuntu/ focal main
EOF
  sudo apt-get -qq update
  sudo apt-get -qq install cmake
  sudo rm /etc/apt/sources.list.d/focal.list
  sudo apt-get -qq update
}

install_ocs2() {
  $(dpkg --compare-versions $(dpkg-query -f='${Version}' --show cmake) gt 3.11) || install_cmake
  $(dpkg --compare-versions $(dpkg-query -f='${Version}' --show cmake) gt 3.11) || fail "CMake >= 3.11 could not be installed"
  git clone --recurse-submodules http://github.com/grizzi/ocs2.git $CATKIN_WS/src/ocs2
  cd $CATKIN_WS/src/ocs2 || fail
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo || fail
  . ~/.bashrc
}

if [ "$DISTRIB_RELEASE" == "18.04" ] && [ "$ROS_DISTRO" == "melodic" ]; then
  echo "${GREEN}Supported distribution was found${NC}"
else
  fail "Your distribution is currently not officially supported"
fi

[ -n "$CATKIN_WS" ] || fail "Please set the CATKIN_WS variable first"

echo "Starting installation"

ROBOTPKG_NAMES=("robotpkg-octomap" "robotpkg-hpp-fcl")
dpkg -s "${ROBOTPKG_NAMES[@]}" >/dev/null 2>&1 || install_robotpkg

[ -d "/usr/local/share/pinocchio" ] || install_pinocchio

[ -d "$CATKIN_WS/src/ocs2" ] || install_ocs2

echo "${GREEN}Installation successful${NC}"
