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
**TODO: add a building script to build the dockers.**  
First, select which of the dockers above you want to use. Then build it with:
```
docker build -f robot.Dockerfile -t ethz-asl/moma_robot .
```
Once the file is built then you can run it with:
```
./run_docker.sh
```

What's important to note:
 - This mounts `~/moma_ws` as `~/moma_ws` inside the docker. Update accordingly depending on where the moma repo is checked out in on your home folder.
 - Mount additional volumes with `--volume /home/$USER/data:/root/data` for example.
 - `--net=host` is very important or the networking won't work

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

### Helen notes
export FRANKA_IP=172.16.0.2

roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=${FRANKA_IP}
