#!/bin/bash
set -o pipefail

cd $MOMA_DEP_WS/src || exit 1

# Install the MOMA, VGN, and ROBOT_HELPERS repos.
vcs import --recursive --input $SCRIPTS_PATH/moma_demo.repos

# Upgrade numpy version
pip install --no-cache-dir --upgrade numpy==1.23.0

# This helps torch download on Github Actions :)
export TMPDIR='/var/tmp'

# Pip install VGN requirements
pip install --no-cache-dir catkin_pkg scipy pandas==0.22.0 matplotlib mpi4py open3d pybullet==2.7.9 tqdm
pip cache purge
pip install --no-cache-dir torch==1.2.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
pip install tensorboard #pytorch-ignite

# Pip install gdown to get google drive files
pip install gdown

# Make sure we can unzip too
apt-get update
apt-get install unzip

# Download the data file from google drive
gdown 1MysYHve3ooWiLq12b58Nm8FWiFBMH-bJ

# Unzip and place it in the correct place
unzip data.zip -d vgn_data
mv vgn_data/data/* vgn/assets/
rm -r vgn_data
rm data.zip

# Add a rundemo alias:
echo 'alias rundemo="roslaunch grasp_demo grasp_demo.launch launch_rviz:=true"' >> ~/.bashrc

# Hope that's it! :)

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
