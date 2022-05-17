cmake_minimum_required(VERSION 3.4)
project(nav_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fPIC)

## Find catkin macros and libraries
## Find other CMake/Catkin packages needed for build
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy roscpp message_generation actionlib geometry_msgs std_msgs sensor_msgs move_base_msgs)

## System dependencies are found with CMake's conventions
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## Enable Python module support 
catkin_python_setup()

## QT setup
set(CMAKE_AUTOMOC ON)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
# add_action_files(DIRECTORY action 
#   FILES 
#   Grasp.action
#   Drop.action
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   actionlib_msgs
#   geometry_msgs
#   sensor_msgs
#   move_base_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${GAZEBO_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_GUI_LIBRARIES} ${CATKIN_LIBRARIES})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

