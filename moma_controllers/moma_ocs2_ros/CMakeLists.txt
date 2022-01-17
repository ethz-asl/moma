cmake_minimum_required(VERSION 3.10)
project(moma_ocs2_ros)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 3.3 REQUIRED)

set(PROJECT_DEPENDENCIES
        roscpp
        realtime_tools
        robot_control
        moma_ocs2
        angles
        tf2
        tf2_ros
        tf2_geometry_msgs
        eigen_conversions
        tf_conversions
        )

find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPENDENCIES})

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

set(DEPENDENCY_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES})

include_directories(include SYSTEM ${DEPENDENCY_INCLUDE_DIRS})

add_library(${PROJECT_NAME} INTERFACE)
#target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})
target_include_directories(${PROJECT_NAME} INTERFACE include)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

# Install library
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
