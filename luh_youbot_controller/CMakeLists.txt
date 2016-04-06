cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_controller)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  kdl_parser
  kdl_conversions
  luh_youbot_driver_api
  luh_youbot_kinematics
  luh_youbot_msgs
  luh_youbot_poses
  roscpp
  tf
  sensor_msgs
  tf_conversions
  cmake_modules
  control_msgs
  luh_youbot_vrep_api
  luh_youbot_gazebo
)
find_package(orocos_kdl REQUIRED)

catkin_package()

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
  include/${PROJECT_NAME}/torque_controller/torque_controller.hpp
  src/torque_controller/youbot_dynamics.cpp
  include/${PROJECT_NAME}/torque_controller/youbot_dynamics.hpp
)

### MODULES ###
add_library(modules 
                 
# base module
src/module_base_class/controller_module.cpp
include/${PROJECT_NAME}/module_base_class/controller_module.h

# interpolation module
src/module_interpolation/module_interpolation.cpp
src/module_interpolation/ramp_generator.cpp
include/${PROJECT_NAME}/module_interpolation/module_interpolation.h
include/${PROJECT_NAME}/module_interpolation/ramp_generator.h

# motion planner module
src/module_motion_planner/motion_planner.cpp
src/module_motion_planner/motion_control.cpp
src/module_motion_planner/motion_planner_callbacks.cpp
src/module_motion_planner/pose.cpp
include/${PROJECT_NAME}/module_motion_planner/motion_planner.h
include/${PROJECT_NAME}/module_motion_planner/motion_control.hpp
include/${PROJECT_NAME}/module_motion_planner/pose.h

# base controller module
src/module_base_controller/base_controller.cpp
include/${PROJECT_NAME}/module_base_controller/base_controller.h

# gripper module
src/module_gripper/gripper.cpp
include/${PROJECT_NAME}/module_gripper/gripper.h

# direct control module
src/module_direct_control/module_direct_control.cpp
include/${PROJECT_NAME}/module_direct_control/module_direct_control.h

# joint trajectory module
src/module_joint_trajectory/module_joint_trajectory.cpp
include/${PROJECT_NAME}/module_joint_trajectory/module_joint_trajectory.h

# gravity compensation module
src/module_gravity_compensation/module_gravity_compensation.cpp
include/${PROJECT_NAME}/module_gravity_compensation/module_gravity_compensation.h
)

### CONTROLLER NODE ###
add_executable(controller_node src/node/main.cpp src/node/controller_node.cpp include/${PROJECT_NAME}/node/controller_node.h)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(controller_node luh_youbot_kinematics)
add_dependencies(torque_controller luh_youbot_kinematics)

target_link_libraries(torque_controller orocos-kdl)
target_link_libraries(modules torque_controller ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(controller_node modules)

## GIVE SUDO RIGHTS TO NODE TO ACCESS ETHERNET INTERFACE
set(EXECUTABLE_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}")


message("=========================================================")
message("Jenkins: ${JENKINS}")

if (${JENKINS})
    message("JENKINS BUILD - Skipping  sudo setcap cap_net_raw+ep controller_node")

else()
    add_custom_command(TARGET controller_node POST_BUILD
        #COMMAND sudo setcap cap_net_raw+ep controller_node
        COMMAND gksudo setcap cap_net_raw+ep controller_node
        WORKING_DIRECTORY ${EXECUTABLE_PATH}
    )
    message("PRODUCTION BUILD - EXECUTING  sudo setcap cap_net_raw+ep controller_node")
endif()


