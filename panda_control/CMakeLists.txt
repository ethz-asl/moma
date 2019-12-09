cmake_minimum_required(VERSION 2.8.3)
project(panda_control)

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
)

find_package(Franka 0.5.0 REQUIRED)

# catkin_python_setup()

catkin_package(
  LIBRARIES franka_example_controllers
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(panda_control
  src/cartesian_velocity_controller.cpp
)

add_dependencies(panda_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(panda_control PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(panda_control SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(panda_control PUBLIC
  include
)

## Installation
install(TARGETS panda_control
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
install(FILES panda_control.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
