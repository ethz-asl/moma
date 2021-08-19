### moma_ocs2

This package provides an interface to ocs2 for control of a mobile manipulator. `ocs2` is a general MPC-DDP solver. In this implementation the robot is represented as a 7-dof mobile manipulator with differential drive base (this can be easily adapted/changed in the future). 

The system dynamics consist in the kinematic model. As a consequence the model predictive controller optimizes the joint velocities and the base twist. The solution is subject to velocity constraints.

### Install

The solver has the potential to check for self collision by specifying which link pair should be checked for self collision. The collision checking library `hpp-fcl` needs therefore to be installed. System modelig (forward kinematics, auto-differentiation and derivatives) uses the `pinocchio` libray that needs to be installed as well. Standard binaries do not work here since, they do not come with support for the collision checking provided by `hpp-fcl`. 

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

Lastly install the packages. These are the tested versions but there are newer. These will be tested later on.

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
First download the [repo](https://stack-of-tasks.github.io/pinocchio/download.html). 
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

Clone this mirror [repository](github.com/grizzi/ocs2.git). Building requires CMake>=3.11. Tested on the panda laptop and since melodic was there at the time, the cmake version is too low. One quick fix is to download a newer CMake binaries and add these at the beginning of the CMAKE_PREFIX_PATH. This step is not required if the system CMake already satisfies the requirements.

#### (optional) Add newer CMake

Download the newer cmake binaries. For example cmake-3.16.3. Assuming that `CMAKE_INSTALL` is the directory containing the binary folders than add the following to the `.bashrc`:
```
export PATH=$HOME/Programs/$CMAKE_INSTALL/bin:$PATH
export CMAKE_PREFIX_PATH=$HOME/$CMAKE_INSTALL:$CMAKE_PREFIX_PATH 
```

### Build

```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build moma_ocs2
```
