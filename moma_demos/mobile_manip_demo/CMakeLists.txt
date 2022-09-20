cmake_minimum_required(VERSION 2.8.3)
project(mobile_manip_demo)

add_compile_options(-std=c++11)

# # Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs geometry_msgs std_msgs)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  MarkerPoses.msg
)

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  ForceDrop.srv
  ForceGrasp.srv
  ObjectDetection.srv
)

# # Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Drop.action Grasp.action Recharge.action
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs # Or other packages containing msgs
  actionlib_msgs
  geometry_msgs
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES fetch_demo
  # CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
)
