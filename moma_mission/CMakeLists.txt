cmake_minimum_required(VERSION 3.10)
project(moma_mission)

find_package(catkin REQUIRED COMPONENTS )

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
        demo/christmas.py
        demo/johnson.py
        demo/mock_modules.py
        demo/piloting.py
        scripts/gripper_commander.py
        scripts/calibrate_door.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
