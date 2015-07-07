cmake_minimum_required(VERSION 2.8.3)
project(luh_youbot_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CartesianPath.msg
  CartesianVector.msg
  CylindricPath.msg
  CylindricVector.msg
  JointPath.msg
  JointVector.msg
  NamedPath.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetArmPose.srv
  GetBasePose.srv
  ListPoses.srv
  ToPredefinedPose.srv
  SetCartesianVelocity.srv
  SetCylindricVelocity.srv
  SetJointVelocity.srv
  GetLoad.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY 
  action
  FILES
  AlignBaseToPose.action
  Empty.action
  GripObject.action
  MoveBase.action
  MoveCartesianPath.action
  MoveCylindricPath.action
  MoveJointPath.action
  MoveNamedPath.action
  MoveToCartesianPose.action
  MoveToCylindricPose.action
  MoveToJointPose.action
  MoveToNamedPose.action
  SetGripper.action
  ForceFit.action
  ApproachBase.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS actionlib_msgs
)
