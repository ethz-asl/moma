cmake_minimum_required(VERSION 2.8.3)
project(panda_control_v2)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  franka_hw
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  message_generation
  std_msgs
)

find_package(Franka 0.5.0 REQUIRED)

catkin_python_setup()

add_message_files(
   FILES
   Command.msg
   FullState.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  
)

catkin_package(
  LIBRARIES franka_example_controllers
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    rospy
    std_msgs
    message_runtime
  DEPENDS Franka
)

add_library(panda_control_v2
  src/constrained_joint_velocity_controller.cpp
)

add_dependencies(panda_control_v2
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(panda_control_v2 PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(panda_control_v2 SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(panda_control_v2 PUBLIC
  include
)

## Installation
install(TARGETS panda_control_v2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES panda_control_v2.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
