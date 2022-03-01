cmake_minimum_required(VERSION 2.8.3)
project(piloting_demo)
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/mock_modules.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

