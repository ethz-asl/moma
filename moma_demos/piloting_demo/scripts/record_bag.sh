#!/bin/bash
rosbag record --repeat-latched \
/tf \
/tf_static \
/base_odom \
/joint_states \
/panda/franka_state_controller/F_ext \
/object_keypoints_ros/result_img \
/valve_path_inverted \
/valve_angle
