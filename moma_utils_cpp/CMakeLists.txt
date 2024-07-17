cmake_minimum_required(VERSION 3.1)
project(moma_utils_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_compile_options(-std=c++17 -Wpedantic -Wall -Warray-bounds)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs moveit_core moveit_ros_planning_interface 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(giraffe_moveit src/giraffe_moveit.cpp)

add_dependencies(giraffe_moveit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(giraffe_moveit
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)