#!/bin/bash
rosbag record --repeat-latched \
/tf \
/tf_static \
/plan_uuid \
/task_uuid \
/sync_id \
/mavsdk_ros/local_position \
/joint_states \
/panda/franka_state_controller/F_ext \
/object_keypoints_ros/result_img \
/valve_continue \
/valve_path_inverted \
/valve_angle \
/photos_taken \
/gauges_read
