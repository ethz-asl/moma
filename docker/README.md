# Dockers

All of these dockers are based heavily on the work of Julian Keller for Piloting.
Not all of them are ready yet; as  rule of thumb, if the file doesn't exist, it's only planned. ;) 


## General Overview
All of these dockers are meant to be used with a local checkout of the moma repo.
The general idea is a 3-level overlay:
1. ROS noetic (in docker image)
2. moma_deps_ws (in docker image)
3. moma_ws (in the host machine, mounted as a volume)

## The images
- **robot.Dockerfile** For the Zotac computer that controls the arm. Includes all necessary sensor drivers (realsense, etc.).
- **dev.Dockerfile** Dev environment (WITH GAZEBO) and without CUDA.
- **dev-cuda.Dockerfile** Same as above but also with CUDA.
- **jetson.Dockerfile** For the robot, for the Jetson. No Gazebo.

## How to use



## How to add new deps
The deps should go into one of several categories: `sys_deps`, `ros_deps`, `drivers`, `simulation`, etc. You can find the `install_*.sh` files in the `scripts` subfolder.
The table below shows which docker images use which files.
The install files are executed in the order shown in the table below. 


| **File**       | **Robot** | **Dev** | **Dev-CUDA** | **Jetson** |
|----------------|:---------:|:-------:|:------------:|:----------:|
| `sys_deps`     |     x     |    x    |       x      |      x     |
| `ros_deps`     |     x     |    x    |       x      |      x     |
| `drivers`      |     x     |         |              |            |
| `simulation`   |           |    x    |       x      |            |
| `build_ros`    |     x     |    x    |       x      |      x     |
