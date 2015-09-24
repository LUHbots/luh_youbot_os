cmake_minimum_required(VERSION 2.8.3)
project(luh_laser_watchdog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  cv_bridge
  image_transport
)

add_message_files(
  FILES
  Distances.msg
)

generate_messages()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(watchdog src/watchdog.cpp include/luh_laser_watchdog/watchdog.h
                        src/footprint_reader.cpp include/luh_laser_watchdog/footprint_reader.h
                        src/collision_watchdog.cpp include/luh_laser_watchdog/collision_watchdog.h
                        src/common.cpp include/luh_laser_watchdog/common.h
                        src/laser.cpp include/luh_laser_watchdog/laser.h)

add_dependencies(watchdog luh_laser_watchdog_generate_messages_cpp)

target_link_libraries(watchdog
  ${catkin_LIBRARIES}
)
