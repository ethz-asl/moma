cmake_minimum_required(VERSION 3.0.2)
project(ur_mpc)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)
find_package(orocos_kdl)

set(PACKAGE_DEPENDENCIES
    angles
    sensor_msgs
    std_msgs
    roscpp
    roslib
    roscpp
    controller_interface
    hardware_interface
    control_toolbox
    moma_ocs2_ros
    path_admittance_controller
    path_passthrough_controller
    tf2_ros
    geometry_msgs
    nav_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

### Ros controller
add_library(${PROJECT_NAME}
            src/ur_mpc.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
