
## Installation

Clone this repository recursively, i.e.

```
git clone --recursive git://github.com/ethz-asl/chimera.git
```

To install ros dependencies through apt-get, run the `install_dependencies.sh` shell script.

## How to run this

To launch a simulation and some standard trajectory following controllers, run

```
roslaunch mopa_control gazebo.launch
```

To launch a simulation together with MoveIt for motion planning, run

```
roslaunch mopa_bringup gazebo_moveit.launch
```

## Setting up the ROS machine with Ansible

The `playbooks/` directory contains some Ansible playbooks which are there to automate and document how to set up the machine. `playbooks/setup_ros.yaml` will install ROS. `playbooks/setup_repo.yaml` will create a catkin workspace and clone this repository on the machine. `playbooks/setup_realsense.yaml` will install realsense dependencies and clone the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) repository.

To run a playbook, first install Ansible (`pip install --user ansible`). Copy the file `playbooks/hosts` to `/opt/ansible/hosts` or specify the inventory file with the `-i` option every time you run `ansible-playbook` (e.g. `ansible-playbook playbooks/setup_ros.yaml -i playbooks/hosts --ask-pass --ask-become-pass`.

The hosts file is an inventory file specifying which machines the playbooks are run against. Currently it only contains one group `franka-box` which's ip should point to the ubuntu machine controlling the Franka arm.

Setting up the machine is done by running the commands:
```
ansible-playbook playbooks/setup_ros.yaml --ask-pass --ask-become-pass
ansible-playbook playbooks/setup_repo.yaml --ask-pass --ask-become-pass
ansible-playbook playbooks/setup_realsense.yaml --ask-pass --ask-become-pass
```

## TODO

- [x] Check if we can move gripper
- [ ] Make second gripper to move the actuated one in gazebo.
- [ ] Fix frames (odom and world are still messed up)
- [x] Set sensible start position in Gazebo simulation
- [x] Fix transform, visual and collision of the realsense in `panda_hand_realsense.urdf.xacro`
- [x] Update `mopa_moveit_config` and bringup launch launch files
