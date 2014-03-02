##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_interactions)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/rocon_interactions
        scripts/interactions_manager.py
        scripts/load_interactions
        scripts/fake_remocon
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
