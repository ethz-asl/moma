cmake_minimum_required(VERSION 3.10)
project(moma_joint_space_controller)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 3.3 REQUIRED)

set(PROJECT_DEPENDENCIES
        actionlib
        moma_msgs
        controller_interface
        hardware_interface
        pluginlib
        roscpp
        robot_control
        realtime_tools
        angles
        actionlib_msgs)


find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPENDENCIES})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

set(DEPENDENCY_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES})

include_directories(include SYSTEM ${DEPENDENCY_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/controller.cpp
        src/trajectory_generator.cpp
        src/velocity_profile.cpp)
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(generator_test unittest/generator_test.cpp)
  target_link_libraries(generator_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(generator_test PRIVATE include)
  add_dependencies(generator_test ${PROJECT_NAME})
endif()

catkin_install_python(PROGRAMS scripts/joint_trajectory_commander.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
