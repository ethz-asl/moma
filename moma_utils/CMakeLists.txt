cmake_minimum_required(VERSION 3.1)
project(moma_utils)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
)

## If you ship Python modules (setup.py present)
catkin_python_setup()

## Declare your service files
add_service_files(
  FILES
  PoseTarget.srv
  GraspTarget.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    rospy
    geometry_msgs
    sensor_msgs
    std_msgs
)

# (Optional) install Python executables if you have any
 catkin_install_python(PROGRAMS
   scripts/panda_grasping_client.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
