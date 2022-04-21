cmake_minimum_required(VERSION 2.8.3)
project(moma_gazebo)

find_package(catkin_simple REQUIRED)

catkin_simple()

catkin_install_python(PROGRAMS scripts/unpause_gazebo.py scripts/gazebo_smart_spawner.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
