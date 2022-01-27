cmake_minimum_required(VERSION 3.10)
project(moma_mission)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_service_files(DIRECTORY srv FILES KeypointsPerception.srv)

catkin_python_setup()

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

catkin_install_python(PROGRAMS
        demo/mock_modules.py
        demo/piloting.py
        scripts/gripper_commander.py
        scripts/calibrate_door.py
        scripts/keypoints_perception.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
