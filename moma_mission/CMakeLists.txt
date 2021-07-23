cmake_minimum_required(VERSION 3.10)
project(moma_mission)

find_package(catkin REQUIRED COMPONENTS )

catkin_package(
        #        INCLUDE_DIRS include
        #        LIBRARIES
        #        CATKIN_DEPENDS
)
catkin_python_setup()

catkin_install_python(PROGRAMS
        demo/mock_modules.py
        demo/piloting_demo.py
        demo/pcl_relay.py
        scripts/gripper_commander.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})