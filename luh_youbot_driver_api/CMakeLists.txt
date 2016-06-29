cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_driver_api)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  youbot_driver
  roslib
  luh_youbot_kinematics
)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES luh_youbot_driver_api
 CATKIN_DEPENDS youbot_driver #geometry_msgs nav_msgs roscpp setcap std_msgs std_srvs tf youbot_driver
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${youbot_driver_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(luh_youbot_driver_api
  include/${PROJECT_NAME}/common.h
  include/${PROJECT_NAME}/youbot_interface.h
  src/youbot_interface.cpp
  include/${PROJECT_NAME}/arm_interface.h
  src/arm_interface.cpp
  include/${PROJECT_NAME}/base_interface.h
  src/base_interface.cpp
  include/${PROJECT_NAME}/arduino_gripper.h
  src/arduino_gripper.cpp
)

add_dependencies(luh_youbot_driver_api luh_youbot_kinematics)
