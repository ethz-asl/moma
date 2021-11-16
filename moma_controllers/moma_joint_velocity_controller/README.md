# moma_joint_velocity_controller

A safety focused velocity controller with a hierarchical objective structure and simulation capabilities, based on [`moma_joint_space_controller`](../moma_joint_space_controller).

## Description

At any point in time, the controller tries to attain the target velocity per joint, while:

1. Respecting the global maximum velocity parameter
2. Starting the deceleration process early enough before hitting joint limits using a constant deceleration model, with a definable safety margin
3. Respecting the global maximum acceleration and deceleration parameters