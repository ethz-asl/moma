# moma_gazebo

Work in progress.

To launch a gazebo simulation and some standard trajectory following controllers, run

```
roslaunch moma_gazebo moma_gazebo.launch robot:=<robot_name>
```
where <robot_name> in {panda, mopa, yumi (TODO), mobmi (TODO)}.


## OLD TODOs (might be outdated)

- [x] Check if we can move gripper
- [ ] Make second gripper to move the actuated one in gazebo.
- [ ] Fix frames (odom and world are still messed up)
- [x] Set sensible start position in Gazebo simulation
