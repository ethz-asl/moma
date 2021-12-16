# moma_robot

This package is intended for launching MoMa components required on the real robot hardware.

## Usage

Launch the main file on the robot PC.
```
roslaunch moma_robot robot_pc.launch
```

For more insights and debugging capabilities during operation, this package can be used in conjunction with the MoMa dashboard, which can be run on the operator PC as follows.
```
roslaunch moma_dashboard dashboard.launch
```