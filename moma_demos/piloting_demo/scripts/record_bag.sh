#!/bin/bash
rosbag record --repeat-latched --output-name=/tmp/2022-04-08_full_demo \
/tf \
/tf_static \
/base_odom \
/joint_states \
/panda/franka_state_controller/F_ext \
/object_keypoints_ros/result_img \
