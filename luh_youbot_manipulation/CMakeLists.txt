cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_manipulation)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  kdl_parser
  kdl_conversions
  luh_youbot_interface
  luh_youbot_kinematics
  luh_youbot_msgs
  luh_youbot_poses
  roscpp
  sensor_msgs
  tf
  tf_conversions
  cmake_modules
  control_msgs
)
find_package(orocos_kdl REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES luh_youbot_manipulation
#  CATKIN_DEPENDS control_toolbox kdl_parser luh_youbot_command_wrapper luh_youbot_kinematics luh_youbot_msgs roscpp sensor_msgs tf tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

### TORQUE CONTROLLER ###
add_library(torque_controller
  src/torque_controller/torque_controller.cpp
  include/luh_youbot_manipulation/torque_controller/torque_controller.hpp
  src/torque_controller/youbot_dynamics.cpp
  include/luh_youbot_manipulation/torque_controller/youbot_dynamics.hpp
)

### MODULES ###
add_library(modules 
                 
# base module
src/module_base_class/manipulation_module.cpp
include/luh_youbot_manipulation/module_base_class/manipulation_module.h

# interpolation module
src/module_interpolation/module_interpolation.cpp
src/module_interpolation/ramp_generator.cpp
include/luh_youbot_manipulation/module_interpolation/module_interpolation.h
include/luh_youbot_manipulation/module_interpolation/ramp_generator.h

# motion planner module
src/module_motion_planner/motion_planner.cpp
src/module_motion_planner/motion_control.cpp
src/module_motion_planner/motion_planner_callbacks.cpp
src/module_motion_planner/pose.cpp
include/luh_youbot_manipulation/module_motion_planner/motion_planner.h
include/luh_youbot_manipulation/module_motion_planner/motion_control.hpp
include/luh_youbot_manipulation/module_motion_planner/pose.h

# base controller module
src/module_base_controller/base_controller.cpp
include/luh_youbot_manipulation/module_base_controller/base_controller.h

# gripper module
src/module_external_gripper/external_gripper.cpp
include/luh_youbot_manipulation/module_external_gripper/external_gripper.h

# external gripper module
src/module_gripper/gripper.cpp
include/luh_youbot_manipulation/module_gripper/gripper.h

# direct control module
src/module_direct_control/module_direct_control.cpp
include/luh_youbot_manipulation/module_direct_control/module_direct_control.h

# joint trajectory module
src/module_joint_trajectory/module_joint_trajectory.cpp
include/luh_youbot_manipulation/module_joint_trajectory/module_joint_trajectory.h

# gravity compensation module
src/module_gravity_compensation/module_gravity_compensation.cpp
include/luh_youbot_manipulation/module_gravity_compensation/module_gravity_compensation.h
)

### MANIPULATION NODE ###
add_executable(manipulation_node src/node/main.cpp src/node/manipulation_node.cpp include/luh_youbot_manipulation/node/manipulation_node.h)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(luh_youbot_manipulation_node luh_youbot_manipulation_generate_messages_cpp)

add_dependencies(manipulation_node luh_youbot_kinematics)
add_dependencies(torque_controller luh_youbot_kinematics)

target_link_libraries(torque_controller orocos-kdl)
target_link_libraries(modules torque_controller ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(manipulation_node modules)

## GIVE SUDO RIGHTS TO NODE TO ACCESS ETHERNET INTERFACE
set(EXECUTABLE_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}")

add_custom_command(TARGET manipulation_node POST_BUILD
    #COMMAND sudo setcap cap_net_raw+ep manipulation_node
    COMMAND gksudo setcap cap_net_raw+ep manipulation_node
    WORKING_DIRECTORY ${EXECUTABLE_PATH}
)
