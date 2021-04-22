# GAZEBO SIMULATION

Launch files, models, worlds and door opening procedure implementation for Gazebo simulation.

## STEPS TO RUN THE LOW LEVEL CONTROL SIMULATION IN GAZEBO

The code is tested using ROS melodic on Ubuntu 18.04.

1. Clone the branch into 'catkin_ws/src' with:
```
git clone --recursive -b projects/articulated-mechanisms-dev https://github.com/ethz-asl/moma.git
```
and run:

```
git submodule update --init
```
to make sure that all submodules are up to date and
```
./install_dependencies.sh
```
to make sure that you have all required packages on your machine.

2. Install 'catkin_simple' package:

```
cd catkin_ws/src
git clone git@github.com:catkin/catkin_simple.git
cd catkin_ws
catkin build catkin_simple
```
3. Install other required packages:

```
sudo apt-get install ros-melodic-lms1xx
catkin build ridgeback_control
sudo apt-get install ros-melodic-interactive-marker-twist-server
catkin build royalpanda_moveit_config
```

4. Run
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
## LOW LEVEL CONTROL FOR DOOR OPENING EXAMPLE

1. Run:

```
roslaunch moma_gazebo royalpanda_example.launch
```

2. Open a new terminal and run:

```
rosrun moma_gazebo panda_state_service
```

3. Open a new terminal and run:

```
rosrun moma_gazebo ROS_door_openning_state_machine_node.py
```


