# Articulated Demo

This was tested on Ubuntu 20.04 with ROS Noetic installed.

## Setup

To install required packages, run

```
moma/moma_demos/articulated_demo/install_dependencies.sh
```

In a ROS Noetic workspace `test_ws/src`, clone the some repos recursively, i.e. using the following commands:

```bash
git clone --recursive -b projects/articulated-mechanisms https://github.com/ethz-asl/moma.git
git checkout projects/articulated-mechanisms
git clone https://bitbucket.org/traclabs/trac_ik/src/master/
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

Build everything:

```
catkin build -DCMAKE_BUILD_TYPE=Release articulated_demo
```

Source the workspace:

```
source test_ws/devel/setup.zsh
```

## Run

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
