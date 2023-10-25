cmake_minimum_required(VERSION 3.5)

project(JointStateModule LANGUAGES CXX)

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

# Make UIC and MOC happy
include_directories(src)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${STATE_MONITOR_MODULE_INSTALL_DIR})

add_library(${PROJECT_NAME}
        src/Inject.cpp
        src/JointStateWidget.cpp
        src/JointStateFrame.cpp
        src/JointStateModule.cpp
        src/JointStateConfig.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Widgets ${catkin_LIBRARIES} yaml-cpp)

catkin_add_gtest(${PROJECT_NAME}_test
        test/test_entry_point.cc
        test/TestJointStateConfig.cpp
)

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} gtest_main)


install(TARGETS ${PROJECT_NAME} DESTINATION ${STATE_MONITOR_MODULE_INSTALL_DIR})