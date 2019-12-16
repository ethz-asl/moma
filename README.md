# MoMa

Main repository of the mobile manipulation (moma) team at ASL containing launch files, robot descriptions, utilities, controllers, and documentation for robot operation.

## Packages

- [`moma_bringup`](moma_bringup/README.md): Launch files and configurations for interfacing with the real robots.
- [`moma_description`](moma_description/README.md): Unified Description Formats (URDFs) of our robots.
- [`moma_gazebo`](moma_gazebo/README.md): Launch files, virtual worlds and plugins for Gazebo simulation (work in progress).
- [`moma_utils`](moma_utils/README.md): Python utilities commonly used within moma projects.
- `panda_control`: Wrappers and custom controllers for controlling panda.
- `panda_moveit_config`: MoveIt configuration for our panda setup.

## Quick Start

Note that all instructions in this repository are tested on Ubuntu 18.04 with ROS melodic.

First, install some general system dependencies.

```
./install_dependencies.sh
```

Clone this repository and its submodules into the `src` folder of a new or existing catkin workspace.

```
git clone --recursive git@github.com:ethz-asl/moma.git
```

Then, use catkin to build the desired packages.

Before you start developing, familiarize yourself with the [robotic platform](https://github.com/ethz-asl/moma/wiki/Robots) you will be working with and make sure to checkout the [Development](https://github.com/ethz-asl/moma/wiki/Development) section of our wiki.

## Documentation

Every package should contain a `README.md` file documenting its main features.

More detailed instructions on the setup and operation of our robotic platforms, as well as some best-practices for development can be found in the [Wiki](https://github.com/ethz-asl/moma/wiki).

## Ansible Playbooks

This section needs to be updated.

The `playbooks/` directory contains some Ansible playbooks which are there to automate and document how to set up the machine. `playbooks/setup_ros.yaml` will install ROS. `playbooks/setup_repo.yaml` will create a catkin workspace and clone this repository on the machine. `playbooks/setup_realsense.yaml` will install realsense dependencies and clone the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) repository.

To run a playbook, first install Ansible (`pip install --user ansible`). Copy the file `playbooks/hosts` to `/opt/ansible/hosts` or specify the inventory file with the `-i` option every time you run `ansible-playbook` (e.g. `ansible-playbook playbooks/setup_ros.yaml -i playbooks/hosts --ask-pass --ask-become-pass`.

The hosts file is an inventory file specifying which machines the playbooks are run against. Currently it only contains one group `franka-box` which's ip should point to the ubuntu machine controlling the Franka arm.

Setting up the machine is done by running the commands:

```
ansible-playbook playbooks/setup_ros.yaml --ask-pass --ask-become-pass
ansible-playbook playbooks/setup_repo.yaml --ask-pass --ask-become-pass
ansible-playbook playbooks/setup_realsense.yaml --ask-pass --ask-become-pass
```
