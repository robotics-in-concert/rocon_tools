##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_python_comms)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()
catkin_add_env_hooks(15.rospair SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks) # tcsh zsh

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/rospair
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(
    FILES
        completion/rospairbash
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/completion
    )

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()