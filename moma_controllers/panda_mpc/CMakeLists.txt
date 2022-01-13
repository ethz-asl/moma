cmake_minimum_required(VERSION 3.0.2)
project(panda_mpc)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)
find_package(Franka 0.7.0 REQUIRED)
find_package(orocos_kdl)

set(PACKAGE_DEPENDENCIES
    angles
    sensor_msgs
    std_msgs
    roscpp
    roslib
    roscpp
    robot_control
    controller_interface
    franka_hw
    hardware_interface
    control_toolbox
    moma_ocs2_ros
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
  DEPENDS Franka
)

include_directories(include ${catkin_INCLUDE_DIRS})

### Ros controller
add_library(${PROJECT_NAME}
        src/panda_mpc.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${Franka_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Franka_LIBRARIES})


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/interactive_target_relay.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})