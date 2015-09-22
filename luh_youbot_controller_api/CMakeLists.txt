cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_controller_api)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  luh_youbot_msgs
  message_generation
  roscpp
  luh_youbot_kinematics
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES controller_api
 CATKIN_DEPENDS luh_youbot_kinematics actionlib actionlib_msgs luh_youbot_msgs message_generation roscpp
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(controller_api
  include/${PROJECT_NAME}/controller_api.h
  include/${PROJECT_NAME}/youbot_arm.h src/youbot_arm.cpp
  include/${PROJECT_NAME}/youbot_base.h src/youbot_base.cpp
  include/${PROJECT_NAME}/youbot_gripper.h src/youbot_gripper.cpp
)

target_link_libraries(controller_api
  ${catkin_LIBRARIES}
)
