
# Fetch Demo

## Launching the demo on the multi-machine setup

### As a single launch file

On the machine used for the user interface, run the following command to distribute the nodes.

```bash
roslaunch fetch_demo single_command.launch
```

### Via SSH

There is one launch file for each machine in the setup. They can all be launch from the user interface laptop via SSH as follows.

Navigation nodes on ridgeback machine:

```bash
ssh asl-admin@asl-ridgeback "source /home/asl-admin/catkin_ws/devel/setup.zsh && roslaunch fetch_demo navigation.launch"
```

Manipulation nodes on YuMi machine:

```bash
ssh asl-admin@asl-yumi "source /home/asl-admin/catkin_ws/devel/setup.zsh && roslaunch fetch_demo manipulation.launch"
```

Perception nodes on moma-laptop:

```bash
ssh asl-admin@moma-laptop "source /home/asl-admin/catkin_ws/devel/setup.zsh && roslaunch fetch_demo perception.launch"
```

On local user interface machine:

```bash
roslaunch fetch_demo user_interface.launch
```

Each of these commands is blocking, they should be run in separate terminals.
