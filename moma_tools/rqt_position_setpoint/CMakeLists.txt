cmake_minimum_required(VERSION 3.0.2)
project(rqt_position_setpoint)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()


#############
## Install ##
#############

# Install resources
install(DIRECTORY resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

# Install python scripts
catkin_install_python(PROGRAMS scripts/rqt_position_setpoint
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Install plugins
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
