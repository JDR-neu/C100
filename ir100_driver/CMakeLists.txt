cmake_minimum_required(VERSION 2.8.3)
project(ir100_driver)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    controller_manager
    hardware_interface
    diagnostic_updater
)

#######tinkerforge#########
set(TINKERFORGE_INCLUDE_DIRS /usr/local/include/tinkerforge)
set(TINKERFORGE_LIBRARY_DIRS -ltinkerforge -lpthread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater hardware_interface roscpp sensor_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(ir100_node
  src/ir100_node.cpp
  src/ir100_diagnostics.cpp
  src/ir100_hw.cpp
  src/motor_controller.cpp
  src/RoboteqDevice.cpp
  src/bms.cpp)
target_link_libraries(ir100_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

add_executable(imu_node
  src/tinkerforge/imu_node.cpp)
target_link_libraries(imu_node
  ${catkin_LIBRARIES}
  ${TINKERFORGE_LIBRARY_DIRS})

add_executable(light_demo
  src/tinkerforge/light_demo.cpp)
target_link_libraries(light_demo
  ${catkin_LIBRARIES}
  ${TINKERFORGE_LIBRARY_DIRS})

add_executable(sonar_demo
  src/tinkerforge/sonar_demo.cpp
  src/tinkerforge/sonar.cpp)
target_link_libraries(sonar_demo
  ${catkin_LIBRARIES}
  ${TINKERFORGE_LIBRARY_DIRS})

install(
    TARGETS
        ir100_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
