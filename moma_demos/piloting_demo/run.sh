#!/bin/bash

tmux kill-session -t PILOTING

if [ ! -z "$1" ]
then
    exit
fi

hostname=192.168.1.12
username=admin
password=987654321
~/catkin_ws/src/franka_lock_unlock/__init__.py $hostname $username $password -u -w -r || exit

top_left=$(tmux new -s PILOTING -P -F "#{pane_id}" -d "bash --rcfile <(echo '. ~/.bashrc; mon launch moma_robot robot_pc_panda.launch hostname:=$(echo $hostname) username:=$(echo $username) password:=$(echo $password)')") || exit
tmux select-pane -t $top_left
bottom_left=$(tmux split-window -P -F "#{pane_id}" -d "bash --rcfile <(echo '. ~/.bashrc; echo Please run roslaunch piloting_demo mission.launch standalone:=true')")
tmux select-pane -t $bottom_left
bottom_right_up=$(tmux split-window -h -P -F "#{pane_id}" -d "bash --rcfile <(echo '. ~/.bashrc; mon launch piloting_demo perception.launch')")
tmux select-pane -t $top_left
top_right_up=$(tmux split-window -h -P -F "#{pane_id}" -d ssh -t piloting@smb261 'bash -ic "mon launch moma_robot robot_pc_smb.launch"; bash -l')
tmux select-pane -t $top_right_up
top_right_down=$(tmux split-window -P -F "#{pane_id}" -d ssh -t piloting@smb261 'bash -ic "mon launch piloting_demo navigation.launch sim:=false"; bash -l')
tmux select-pane -t $bottom_right_up
bottom_right_down=$(tmux split-window -P -F "#{pane_id}")

echo Attach using "tmux a -t PILOTING"
