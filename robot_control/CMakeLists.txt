cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_srvs
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(angle_filter src/angle_filter_node.cpp)
add_executable(control src/control_node.cpp)
add_executable(ui src/ui_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(angle_filter ${catkin_LIBRARIES})
target_link_libraries(control ${catkin_LIBRARIES})
target_link_libraries(ui ${catkin_LIBRARIES})

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
  launch/robot_launch.xml
# #   # myfile2
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
