cmake_minimum_required(VERSION 3.10)
project(moma_ocs2_ros)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 3.3 REQUIRED)

set(PROJECT_DEPENDENCIES
        roscpp
        realtime_tools
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
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

set(DEPENDENCY_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES})

include_directories(include SYSTEM ${DEPENDENCY_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/mpc_velocity_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

# Install library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
