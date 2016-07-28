cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  roslib
  luh_youbot_kinematics
  luh_youbot_driver_api
  gazebo_ros
  luh_youbot_gripper
)

find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES luh_youbot_gazebo_interface
 DEPENDS gazebo_ros
         geometry_msgs
         sensor_msgs
         nav_msgs
         roscpp
         tf
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### interface ###
add_library(luh_youbot_gazebo_interface
  include/${PROJECT_NAME}/youbot_interface.h
  src/youbot_interface.cpp
  include/${PROJECT_NAME}/arm_interface.h
  src/arm_interface.cpp
  include/${PROJECT_NAME}/base_interface.h
  src/base_interface.cpp
)
add_dependencies(luh_youbot_gazebo_interface luh_youbot_driver_api)

### plugins ###
add_library(youbot_base_controller src/plugins/base_controller.cpp include/${PROJECT_NAME}/plugins/base_controller.h)
target_link_libraries(youbot_base_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(youbot_arm_controller src/plugins/arm_controller.cpp include/${PROJECT_NAME}/plugins/arm_controller.h)
target_link_libraries(youbot_arm_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(youbot_gripper_controller src/plugins/gripper_controller.cpp include/${PROJECT_NAME}/plugins/gripper_controller.h)
target_link_libraries(youbot_gripper_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(youbot_joint_state_publisher src/plugins/joint_state_publisher.cpp include/${PROJECT_NAME}/plugins/joint_state_publisher.h)
target_link_libraries(youbot_joint_state_publisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
