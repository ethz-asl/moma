# moma_controllers

This directories contains all ROS packages implementing a controller for the robot.

### Implemented controllers

- **moma_joint_space_controller** : drives the arm from the current to a desired angular position using a trapezoidal velocity profile.
- **moma_ocs2** : implementation of a whole body kinematic MPC controller using the ocs2 library.
- **moma_ocs2_ros** : wrapper of the controller implemented in `moma_ocs2`. Implements all methods to set and modify a target pose to be tracked by the end effector.
- **panda_mpc** : ROS controller using `moma_ocs2_ros` and the ROS control framework to control the panda robot.
- **path_admittance_controller** : this controller serves only as a relay for MPC. It takes an input path trajectory and modifies it according to the latest received wrench from the end effector. It implements the admittance dynamics.

### Notes and TODOs

- The `moma_ocs2_ros` package could be simply part of the `moma_ocs2` as there is no particular reason to have them separately. In fact, the `moma_ocs2` is not ROS agnostic as the name could suggest.
- The mpc implementation accounts for the fact that the robot is mounted on the mobile base and that therefore the full command space includes the base twist (assumed non-holonomic) (see the [configuration file](moma_ocs2/config/mpc/task_panda.info)). At the moment this is only used and tested for control of the arm. This is achieved by setting to 0 the velocity limits of the base and extrapolating only arm commands in `panda_mpc`.
