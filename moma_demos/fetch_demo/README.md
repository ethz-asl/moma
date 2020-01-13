
# Fetch Demo

## Launching the demo on the multi-machine setup

### Via SSH

There is one launch file for each machine in the setup. They can all be launch from the user interface laptop via SSH as follows.

Navigation nodes on ridgeback machine:

```bash
ssh -t asl-admin@asl-ridgeback "source /home/asl-admin/.zshrc && roslaunch fetch_demo navigation.launch"
```

Manipulation nodes on YuMi machine:

```bash
ssh -t asl-admin@asl-yumi "source /home/asl-admin/.zshrc && roslaunch fetch_demo manipulation.launch"
```

Perception nodes on moma-laptop:

```bash
ssh -t asl-admin@moma-laptop "source /home/asl-admin/.zshrc && roslaunch fetch_demo perception.launch"
```

On local user interface machine:

```bash
roslaunch fetch_demo user_interface.launch
```

Each of these commands is blocking, they should be run in separate terminals.

### As a single launch file

On the machine used for the user interface, run the following command to distribute the nodes.

```bash
roslaunch fetch_demo single_command.launch
```

This only works if all nodes in this launch file are also installed on the local machine.


## Useful commands

### Test the approach node

Send goals directly to move_base:

``bash
rosrun actionlib axclient.py move_base move_base_msgs/MoveBaseAction
```

Send goals to the approach action:

```bash
rosrun actionlib axclient.py approach_action fetch_demo/ApproachAction
```
