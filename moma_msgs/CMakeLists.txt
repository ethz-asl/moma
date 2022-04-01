cmake_minimum_required(VERSION 3.0.2)
project(moma_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  actionlib_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES Joint.action
)

generate_messages(
 DEPENDENCIES
  actionlib_msgs geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
