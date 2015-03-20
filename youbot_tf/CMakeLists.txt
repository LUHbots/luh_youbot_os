cmake_minimum_required(VERSION 2.8.3)
project(youbot_tf)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  robot_state_publisher
  trajectory_msgs
)

catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_odom src/tf_odom.cpp)
target_link_libraries(tf_odom ${catkin_LIBRARIES})
