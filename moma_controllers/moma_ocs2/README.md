### moma_ocs2

This package provides an interface to ocs2 for control of a mobile manipulator. `ocs2` is a general MPC-DDP
solver. In this implementation the robot is represented as a 7-dof mobile manipulator with differential drive base (this can be easily adapted/changed in the future). 

The system dynamics consist in the kinematic model. As a consequence the model predictive controller optimizes the joint velocities and the base twist. The solution is subject to velocity constraints.

### Install

#### Install hpp-fcl
Add the `robotpkg` repository to the available source. Find the system you are working on:
```
lsb_release -c`
```

In this case assume it is `bionic`. Then update the repositories:

```
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub bionic robotpkg
EOF
```

Add the key:

```
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |
   sudo apt-key add -
```

Finally run a update to fetch all the information:

```
sudo apt-get update
```

Lastly install the packages. These are the tested versions. There are newer. These will be tested later on.

```
sudo apt-get install robotpkg-octomap=1.6.1
sudo apt-get install robotpkg-hpp-fcl=1.6.0
```

Then add the following lines to the `.bashrc` file and source it such that depending packages can find the collision detection library.

```
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

#### Install pinocchio

Pinocchio does not come with default hpp-fcl integration and thus needs to be installed from source. 
First download the repo. 
```
https://stack-of-tasks.github.io/pinocchio/download.html
```

The code has been tested for the current version that you need to checkout:
```
git checkout 29be057af1beb

```
After cloning, create a build directory and follow these instructions
```
mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local/install -DBUILD_WITH_COLLISION_SUPPORT=ON
make -j4
make install 
```

Add the installation to your paths in the `.bashrc` file.
```
export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/dist-packages
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```
#### Install ocs2

You must first ask access to the `ocs2_dev` branch sending an email to (fill) with your bitbucket email address and account name. After being added to the ASL user group, you can clone the `ocs2_dev` in your catkin 
workspace. Make sure to checkout the latest tested commit:

```
git clone (to do)
git checkout a813a4dc2d1f
```

### Build

```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build moma_ocs2
```
