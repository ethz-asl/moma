cmake_minimum_required(VERSION 3.4)
project(panda_test)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Franka 0.7.0 REQUIRED)

add_message_files(FILES
  command_msg.msg
)

add_service_files(FILES
  PandaStateSrv.srv
  PandaGripperSrv.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES panda_test
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(panda_test
  src/door_opening_joint_velocity_controller.cpp
)

add_dependencies(panda_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)

target_link_libraries(panda_test PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(panda_test SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(panda_test PUBLIC
  include
)

add_executable(panda_state_service
  src/panda_state_service.cpp
)
target_include_directories(panda_state_service PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(panda_state_service PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(panda_state_service
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)

add_executable(panda_gripper_service
  src/panda_gripper_service.cpp
)
target_include_directories(panda_gripper_service PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(panda_gripper_service PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(panda_gripper_service
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)

## Installation
install(TARGETS panda_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES panda_test_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(panda_test FILES ${SOURCES} ${HEADERS})
  add_tidy_target(panda_test
    FILES ${SOURCES}
    DEPENDS panda_test
  )
endif()
