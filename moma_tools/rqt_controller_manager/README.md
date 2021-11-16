# RQT Controller Manager
## Improved Version

This version is an adaptation which has several improvements over the current default ROS melodic [`rqt_controller_manager`](http://ros.org/wiki/rqt_controller_manager).
- Default selection of `/controller_manager` topic on start.
- Controller sorting by name.
- Switching between multiple controllers by simultaneously selecting all of them (for example by holding CTRL) and then triggering the "Switch" action.
- Automatic detection of incompatible controllers by means of claimed hardware resources and proposal of stopping the incompatible ones on start of a new controller.