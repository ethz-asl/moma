cmake_minimum_required(VERSION 3.0.2)
project(articulated_demo)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
