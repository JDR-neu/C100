################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(ir100_navigation)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED)

catkin_package()

################################################################################
# Install
################################################################################
install(DIRECTORY launch maps param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

