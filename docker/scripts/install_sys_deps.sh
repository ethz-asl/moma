# Update Ubuntu packages to latest.
apt-get -qq update && apt-get -qq upgrade

# update git and set to always point to https
apt-get -qq update && apt-get install -y curl
apt-get -qq update && apt-get install -y git git-lfs
git config --global url.https://github.com/.insteadOf git@github.com:

# get install tools
apt-get -qq update && apt-get install -y python3-catkin-tools python3-vcstool python3-pip

# Install other system deps
apt-get -qq update && apt-get install -y 	qtbase5-dev