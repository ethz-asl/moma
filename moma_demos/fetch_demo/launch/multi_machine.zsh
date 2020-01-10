#!/bin/zsh

ssh asl-admin@asl-ridgeback "source /home/asl-admin/catkin_ws/devel/setup.zsh && roslaunch fetch_demo navigation.launch"

ssh asl-admin@asl-yumi "source /home/asl-admin/catkin_ws/devel/setup.zsh && roslaunch fetch_demo manipulation.launch"

ssh asl-admin@moma-laptop "source /home/asl-admin/catkin_ws/devel/setup.zsh && roslaunch fetch_demo perception.launch"

# On local user interface machine
roslaunch fetch_demo user_interface.launch
