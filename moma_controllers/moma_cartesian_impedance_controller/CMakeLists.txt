cmake_minimum_required(VERSION 3.4)
project(moma_cartesian_impedance_controller)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_DEPENDENCIES
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  pluginlib
  realtime_tools
  roscpp
  robot_control)

find_package(catkin REQUIRED COMPONENTS
  ${PROJECT_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
  DEPENDS Franka
)

add_library(${PROJECT_NAME}
  src/cartesian_impedance_controller.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

## Installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/recover.py scripts/set_EE_frame.py scripts/interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
