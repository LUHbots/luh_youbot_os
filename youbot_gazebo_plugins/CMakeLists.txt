cmake_minimum_required(VERSION 2.8.3)
project(youbot_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  tf
)

find_package(gazebo REQUIRED)

catkin_package(
   DEPENDS   gazebo_ros
             geometry_msgs
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

add_library(youbot_base_controller src/base_controller.cpp include/youbot_gazebo_plugins/base_controller.h)
target_link_libraries(youbot_base_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
