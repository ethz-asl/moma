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

## GPD

When building a package that relies on GPD (e.g. `fetch_demo`), GPD needs to be built separately first. For the installation instructions, refer to `submodules/gpd/README.md`.

## Documentation

Every package should contain a `README.md` file documenting its main features.

More detailed instructions on the setup and operation of our robotic platforms, as well as some best-practices for development can be found in the [Wiki](https://github.com/ethz-asl/moma/wiki).

## Ansible Playbooks

The `operations/` directory contains some Ansible playbooks which are there to automate and document how to set up a computer.

To run a playbook, first install Ansible (`pip install --user ansible`). Copy the hosts file `operations/hosts` to `/etc/ansible/hosts`.

The hosts file is an inventory file specifying groups of hosts. A playbook is always run against a group and all the commands will be run on all machines in that group.

To set up the franka control computer, run:
```
ansible-playbook operations/setup_franka.yaml --ask-pass --ask-become-pass
```
The `--ask-pass` flag is not needed if ssh authentication is being used.

