# stacking_demo

## Install

## Instructions

For the following instructions, it is assumed that the user is logged into `asl-panda` and has sourced the catkin workspace containing the demo package.


Before you start, make sure to have set the correct tower_height in the config/stacking_demo.yaml file.


First, Launch the nodes:

```bash
roslaunch stacking_demo stacking_demo.launch
```

To pick up the object from the object_position:
```
rosservice call /move_to_object
``` 

To go to the tower and place the object, call the service:

```
rosservice call /move_to_tower "y_offset: 0.0"
```
## Local Docker setup:
On the static panda, here is how to run the docker for this code:
```
cd /home/franka/Projects/lucy/robotx_ws/src/moma/docker
./run_docker.sh -d robot.Dockerfile -d robotx -w ~/Projects/lucy/robotx_ws
```
To open a new terminal in docker:
```
docker exec -it moma bash
```


## Debugging

For debugging, 
To close the gripper, publish in terminal the following command: for opening, set the width to 0.0 
```
rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.3
  speed: 0.1"
```

If you want to bring up the robot without the stacking:
```bash
roslaunch moma_bringup panda_real.launch
```