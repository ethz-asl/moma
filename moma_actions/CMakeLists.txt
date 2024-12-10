cmake_minimum_required(VERSION 3.0.0)
project(moma_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_action_files(
  FILES
  Drop.action
  Grasp.action
  Move.action
  Trigger.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  message_runtime
  rostest
)

include_directories(
  ${caktin_INCLUDE_DIRS}
)