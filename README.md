# MoMa

[![CI](https://github.com/ethz-asl/moma/actions/workflows/main.yml/badge.svg)](https://github.com/ethz-asl/moma/actions/workflows/main.yml)

Main repository of the mobile manipulation (moma) team at ASL containing launch files, robot descriptions, utilities, controllers, and documentation for robot operation.

## Packages

- [`moma_bringup`](moma_bringup/README.md): Launch files and configurations for interfacing with the real robots.
- [`moma_description`](moma_description/README.md): Unified Description Formats (URDFs) of our robots.
- [`moma_gazebo`](moma_gazebo/README.md): Launch files, virtual worlds and plugins for Gazebo simulation (work in progress).
- [`moma_tools`](moma_tools/README.md): User interfaces and tools for real-time robot state visualization and interaction.
- [`moma_utils`](moma_utils/README.md): Python utilities commonly used within moma projects.
- `panda_control`: Wrappers and custom controllers for controlling panda.
- `panda_moveit_config`: MoveIt configuration for our panda setup.

## Quick Start

Note that all instructions in this repository are tested on Ubuntu 18.04 with ROS melodic.

First, clone this repository and its submodules into the `src` folder of a new or existing catkin workspace.

```bash
git clone --recurse-submodules git@github.com:ethz-asl/moma.git
```

Set the environment variable `$CATKIN_WS` to the root of your catkin workspace.

```bash
export CATKIN_WS=~/catkin_ws
```

To download the dependency packages, the vcs command-line tools will be used. For more information about the tool you can check this [link](http://wiki.ros.org/vcstool).

```bash
sudo apt install python3-vcstool
```

To download the moma packages by using vcs tool run the following terminal commands in order.

```
cd $CATKIN_WS/src
vcs import --recursive --input moma/moma_core.repos
```

To install additional packages (as for example required for the demos or piloting), use `vcs`:

```
vcs import --recursive --input moma/moma_<NAME>.repos
```

where `<NAME>` needs to be replaced by the corresponding repos filename.

Install the remaning dependencies using the provided script.

```bash
cd $CATKIN_WS/src
./moma/install_dependencies.sh
```

Ensure to build in `RelWithDebInfo` mode. Otherwise, many controllers are very slow.

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Wiki

Before you start developing, familiarize yourself with the [robotic platform](https://github.com/ethz-asl/moma/wiki/Robots) you will be working with and make sure to checkout the [development](https://github.com/ethz-asl/moma/wiki/Development) section of our wiki.

### Grasp demo

**Update needed**

~~Then, use catkin to build the desired packages, e.g.~~

```bash
catkin build grasp_demo
```

~~When building a package that relies on GPD (e.g. `fetch_demo` or `grasp_demo`), GPD needs to be built separately first. For the installation instructions, refer to `submodules/gpd/README.md`.~~

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
