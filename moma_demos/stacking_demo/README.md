# stacking_demo

## Object setup
You have to manually build up the tower in the beginning. It is important that the tower is correctly build, since this is hardcoded. The picture below shows how it should look like, the positions for the objects are [0, 0.02, 0.045] m for the geometric center of the three objects (wood cube, alu+wood+wood, wood cube). If it is not stable enough, move the aluminium cube a bit to the left.
The three objects to be stacked (blue+blue, blue+green, blue+alu) have to be correctly placed on the printed A3 sheet from config/colored_objs.pdf
The robot will randomly select one of the 3 objects, get a prediction for placement, place it, return it. It runs in an infinite loop.
Since the robot is not very repeatable, you will have to once in a while make sure that the objects are actually were they should be on the paper. Also, the robot might move the tower around a bit, then you can also manually straigthen it up. 
If the robot crashes into something or you have to stop it with the e-stop, you should also stop the script on the computer, bring the robot back in the blue light mode and then start the script again.

![Tower setup](https://github.com/ethz-asl/moma/tree/feature/bota/moma_demos/stacking_demo/config/tower.jpg "Tower setup")


## Instructions to run

For the following instructions, it is assumed that the user is logged into `asl-panda` and has sourced the catkin workspace containing the demo package.


Before you start, make sure to have set the correct tower_height in the config/stacking_demo.yaml file and the launch/load_static_tf.launch file.


Launching the node will automatically start the state machine:

```bash
roslaunch stacking_demo stacking_demo.launch
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

To test the stacking functions without the state machine running, you can comment the run_plan.py node in the stacking_demo.launch file and instead manually call the services, e.g.

To pick up the object from the object_position:
```
rosservice call /move_to_object
``` 

To go to the tower and place the object, call the service:

```
rosservice call /move_to_tower "y_offset: 0.0"
```

## Changing the pose predictor
In case you want to change the tower setup or the objects to place, refer to Paula's position predictor repo to retrain.
https://github.com/paulawulkop/RobotX_stacking/tree/stacking_task 
