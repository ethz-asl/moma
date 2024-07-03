cmake_minimum_required(VERSION 3.1)
project(moma_utils_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning_interface
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs moveit_core moveit_ros_planning_interface 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(giraffe_panda_safe src/giraffe_panda_safe.cpp)
add_dependencies(giraffe_panda_safe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(giraffe_panda_safe
  ${catkin_LIBRARIES}
)