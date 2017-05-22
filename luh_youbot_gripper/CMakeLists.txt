cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_gripper)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  roslib
  sensor_msgs
  actionlib
  luh_youbot_msgs
)

add_service_files(
  FILES
  GripCheck.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  DEPENDS luh_youbot_msgs
)

###########
## Build ##
###########


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(luh_gripper_node
               src/main.cpp
               src/gripper.cpp
               include/luh_youbot_gripper/gripper.h
               src/gripper_node.cpp
               include/luh_youbot_gripper/gripper_node.h)

target_link_libraries(luh_gripper_node
  ${catkin_LIBRARIES}
)

add_dependencies(luh_gripper_node ${catkin_EXPORTED_TARGETS})

add_executable(gripper_test_node
               src/test_node.cpp)

target_link_libraries(gripper_test_node
  ${catkin_LIBRARIES}
)

add_dependencies(gripper_test_node ${catkin_EXPORTED_TARGETS})
