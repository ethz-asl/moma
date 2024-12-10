cmake_minimum_required(VERSION 3.1)
project(moma_utils)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/moma_utils/ros/panda_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
