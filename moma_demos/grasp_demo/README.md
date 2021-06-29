# grasp_demo

## Install

## Instructions

For the following instructions, it is assumed that the user is logged into `asl-panda` and has sourced the catkin workspace containing the demo package.

Frist, launch the hardware drivers and nodes.

```bash
roslaunch grasp_demo grasp_demo.launch [semantic:=true] [detect_grasps_with:=vgn]
```

* To run the demo in Gazebo, add `simulation_mode:=true` to the above command.
* If `semantic:=true` is set, make sure to also launch voxblox++ on `asl-dell` with `roslaunch gsm_node panda.launch`.
* `detect_grasps_with:=vgn` requires `semantic:=true` as it relies on the TSDF map from voxblox++.

Next, for interacting with the demo through Rviz, run 

```
 rosrun grasp_demo run_bt.py __ns:=manipulator
```

Or alternatively, to run the demo continuously, use

```
rosrun grasp_demo run_plan.py __ns:=manipulator
```

## Troubleshooting

- Make sure that `ROS_MASTER_URI` is properly set in all terminal sessions.
- Restart the ROS core.

## To Do

- [ ] Test voxblox++
- [ ] Semantic grasp selection
