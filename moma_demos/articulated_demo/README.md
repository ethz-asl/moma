# Articulated Demo

This was tested on Ubuntu 20.04 with ROS Noetic installed.

## Setup

### Dependencies

To install required packages, run

```
moma/moma_demos/articulated_demo/install_dependencies.sh
```

Furthermore, the package `robot_control` requires Pinocchio to build, so install this as per the instructions here: https://stack-of-tasks.github.io/pinocchio/download.html. Remember to replace the python version in "robotpkg-py27-pinocchio" with the one you have.

In a ROS Noetic workspace `test_ws/src`, clone some repos recursively, i.e. using the following commands:

```bash
git clone --recursive -b projects/door_opening https://github.com/ethz-asl/moma.git
git clone https://bitbucket.org/traclabs/trac_ik
git clone --recursive -b cartesian-velocity-controller https://github.com/ethz-asl/robot_control.git
```

Links to the repos:

- [moma](https://github.com/ethz-asl/moma)
- [trac-ik](https://bitbucket.org/traclabs/trac_ik/src/master/)
- [robot_control](https://github.com/ethz-asl/robot_control)

Source ROS:

```
source /opt/ros/noetic/setup.zsh
```

### Build

Build everything:

```
catkin build -DCMAKE_BUILD_TYPE=Release articulated_demo
```

If an error message comes up when building `trac_ik`, complaining about the header `nlopt.hpp` missing, it helps to install nlopt from source: https://github.com/stevengj/nlopt.

Source the workspace:

```
source test_ws/devel/setup.zsh
```

### Virtual Environment

Finally, before running, some python dependencies need to be installed. It is good practice to do this in a virtual environment, to keep dependencies separated from other projects. Use these commands to create one:

```zsh
cd test_ws/src/moma
virtualenv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Note that you should not invoke `catkin build` in a terminal where this virtual environment is activated, to make sure that all ROS related python packages end up in the system installation of python, which is the one that ROS uses.

## Run

Before running, make sure that your environment is properly set up, i.e. that ROS, the workspace and the virtual environment are sourced:

```zsh
source /opt/ros/noetic/setup.zsh
source ~/test_ws/devel/setup.zsh
source ~/test_ws/src/moma/.venv/bin/activate
```

To launch the demo, run

```zsh
python moma/moma_demos/articulated_demo/scripts/run_manually.py -m gui
```

If you want to use the shared memory mode, build bullet from source, run `App_ExampleBrowser`, and then use the option `-m shared` with above command. After running once in shared memory mode, you can reuse the existing session on the next call of the demo by passing the `-r` option. With this, the models don't have to be reloaded, which speeds up the initialization at lot.

To run the door opening PyBullet experiment for one particular initial configuration of the robot, run:

```
python moma/moma_demos/articulated_demo/scripts/run_single_configuration.py -m gui
```

To run the door opening PyBullet experiment for multiple initial configurations of the robot, run:

```
python moma/moma_demos/articulated_demo/scripts/run_multiple_configurations.py -m gui
```

Choosing the initial pose parameters and door type is performed in the 'main' function of the corresponding scripts. 
