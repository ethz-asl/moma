cmake_minimum_required(VERSION 3.0.2)
project(arche_2025)

find_package(catkin REQUIRED COMPONENTS
  moma_bringup
  franka_ros
  cartesian_impedance_controller
)

catkin_package()