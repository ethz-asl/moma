# Install Gazebo
apt-get -qq update && apt-get install -y ros-noetic-gazebo-ros 

# Install Franka stuff
apt-get -qq update && apt-get install -y ros-noetic-franka-ros ros-noetic-franka-gazebo

# Install remaining packages
cd $MOMA_DEP_WS/src
vcs import --recursive --input $SCRIPTS_PATH/moma_simulation.repos
