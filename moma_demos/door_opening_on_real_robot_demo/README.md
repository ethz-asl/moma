## STEPS TO RUN THE CODE ON THE REAL ROBOT:

1. Build the "door_opening_on_real_robot_demo" package on the Franka computer and on the personal computer. 
The ROS control plugin and the ROS services have to run on the Franka computer whereas the script for the state machine should run on the personal computer due to use of the third party libraries. 
The ROS Master should be located on the Ridgeback base and should be running all the time.

2. Run:

```
roslaunch panda_test joint_velocity_controller_full.launch robot_ip:=$FRANKA_IP load_gripper:=false
```

on the Franka computer. The robot should be in the 'high level API' state (Lights on the arm should be white). 
It will complain that the automatic control is not possible in the current regime but for now, this is the right thing to do.

3. Open a new terminal and run:

```
rosrun panda_test panda_gripper_service $FRANKA_IP
```
on the Franka computer.

4. Open a new terminal and run:

```
rosrun panda_test panda_state_service $FRANKA_IP
```

on the Franka computer.

5. Open a new terminal and run:

```
rosrun panda_test ROS_door_opening_state_machine_testing_node.py
```

on the personal computer. This node will require some user input. The answer to each question has to be entered with quotation marks (e.g. 'y' or 'n'). 
After the state machine informs the user that the EE frame and the K frame have been set, the user should re-launch the joint_velocity_controller.launch file but this time with the Franka arm in the automatic control mode (Lights on the arm should be blue). 
This is neccessary because the process of setting the EE and the K frames is done through the non-realtime commands of the specialized 'libfranka' library that cannot be used in the automatic regime. 
The reset should be completed before entering the response to the 'Initiate door opening procedure?' question. From this point on, the user can proceed normally.

