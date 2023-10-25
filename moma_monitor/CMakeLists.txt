cmake_minimum_required(VERSION 3.5)

project(moma_monitor LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        state_monitor_sdk
        )

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS sensor_msgs state_monitor_sdk
        DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

set(STATE_MONITOR_MODULE_INSTALL_DIR ~/.state_monitor/modules)

add_subdirectory(JointStateModule)