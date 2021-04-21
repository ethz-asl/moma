# GAZEBO SIMULATION

Launch files, models, worlds and door opening procedure implementation for Gazebo simulation.

## STEPS TO RUN THE LOW LEVEL CONTROL SIMULATION IN GAZEBO

The code is tested using ROS melodic on Ubuntu 18.04.

1. Run:

```
git submodule update --init
```
to make sure that all submodules are up to date and
```
./install_dependencies.sh
```
to make sure that you have all required packages on your machine.

2. Run:

```
catkin build moma_gazebo
```

## PANDA EXAMPLE

To run the demo:

```bash
roslaunch moma_gazebo panda_example.launch
```

## ROYAL PANDA EXAMPLE

To run the demo:

```bash
roslaunch moma_gazebo royalpanda_example.launch
```
## DOOR OPENING EXAMPLE


