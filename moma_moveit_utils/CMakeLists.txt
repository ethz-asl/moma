cmake_minimum_required(VERSION 2.8.3)
project(moma_moveit_utils)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
