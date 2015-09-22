cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  luh_youbot_controller_api
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_library(NLOPT_FOUND nlopt)
if(NOT NLOPT_FOUND)
message(STATUS "### WARNING###\nNLOPT library is not installed. luh_youbot_calibration will not be built.")
endif(NOT NLOPT_FOUND)

if(NLOPT_FOUND)

add_executable(static_calibration src/static_calibration.cpp)

target_link_libraries(static_calibration
  ${catkin_LIBRARIES}
  yaml-cpp
  nlopt
)

endif(NLOPT_FOUND)
