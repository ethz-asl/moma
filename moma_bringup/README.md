# moma_bringup

This package contains launch files to start interfaces with the real robots.
See the [wiki](https://github.com/ethz-asl/moma/wiki/Robots) for more information about our platforms.
### Running handeye calibration (currently only in sim)

Spawn the panda robot which contains an april tag:
```console
roslaunch moma_gazebo panda_moma_corner.launch
```

Then launch the interactive calibration planner:
```console
roslaunch moma_bringup move_group_sphere.py
```
