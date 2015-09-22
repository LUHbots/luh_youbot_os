/* *****************************************************************
 *
 * luh_youbot_controller
 *
 * Copyright (c) 2015,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/

#ifndef LUH_YOUBOT_CONTROLLER_MOTIONPLANNER_H
#define LUH_YOUBOT_CONTROLLER_MOTIONPLANNER_H

#include "ros/ros.h"
#include "../module_base_class/controller_module.h"
#include "../module_motion_planner/pose.h"
#include "../module_motion_planner/motion_control.hpp"
#include "luh_youbot_msgs/ToPredefinedPose.h"
#include "luh_youbot_msgs/ListPoses.h"
#include "actionlib/server/simple_action_server.h"
#include "luh_youbot_msgs/MoveCartesianPathAction.h"
#include "luh_youbot_msgs/MoveCylindricPathAction.h"
#include "luh_youbot_msgs/MoveJointPathAction.h"
#include "luh_youbot_msgs/MoveNamedPathAction.h"
#include "luh_youbot_msgs/MoveToCartesianPoseAction.h"
#include "luh_youbot_msgs/MoveToCylindricPoseAction.h"
#include "luh_youbot_msgs/MoveToJointPoseAction.h"
#include "luh_youbot_msgs/MoveToNamedPoseAction.h"
#include "luh_youbot_controller/torque_controller/torque_controller.hpp"
#include "luh_youbot_controller/torque_controller/youbot_dynamics.hpp"

typedef std::deque<luh_youbot_kinematics::JointPosition> Path;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveJointPathAction>     JointPathServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveCartesianPathAction> CartesianPathServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveCylindricPathAction> CylindricPathServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveNamedPathAction>     NamedPathServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToCartesianPoseAction>     CartesianPoseServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToCylindricPoseAction>     CylindricPoseServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToJointPoseAction>         JointPoseServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToNamedPoseAction>         NamedPoseServer;

class PoseSet : public std::vector<Pose*>
{
public:
    PoseSet();
    Pose* getMinCostPose();
};

class ModuleMotionPlanner : public ControllerModule
{
public:

    enum CommandMode
    {
        POSITION,
        VELOCITY,
        TORQUE
    };

    ModuleMotionPlanner();
    ~ModuleMotionPlanner();

    Path getPath(luh_youbot_kinematics::JointPosition start, luh_youbot_kinematics::JointPosition end);
    Pose* findClosestPose(luh_youbot_kinematics::JointPosition pose);

protected:

    ros::ServiceServer predef_pose_server_;
    ros::ServiceServer list_poses_server_;
//    ros::Publisher position_publisher_;
    ros::Publisher position_error_publisher_;

    JointPathServer* joint_path_server_;
    CartesianPathServer* cartesian_path_server_;
    CylindricPathServer* cylindric_path_server_;
    NamedPathServer* named_path_server_;
    CartesianPoseServer* cartesian_pose_server_;
    CylindricPoseServer* cylindric_pose_server_;
    JointPoseServer* joint_pose_server_;
    NamedPoseServer* named_pose_server_;

    motionPlanning* controller_;

    luh_youbot_kinematics::JointVelocity joint_velocities_;

    std::vector<Pose> poses_; /// these poses are the nodes in the graph
//    std::vector<Pose> additional_poses_; /// additional poses that can be set as goals
    std::map<std::string, Pose*> pose_map_;
    luh_youbot_kinematics::JointPosition end_pose_;

    double euclidean_influence_factor_; /// determines the influence of the euclidean distance for the costs
    double constant_influence_;
    luh_youbot_kinematics::JointVector joint_weights_;

    bool deactivated;

    double velocity_factor_;

    double finepos_pos_tolerance_;
    double finepos_vel_tolerance_;

    nAxesControllerTorque* torque_controller_;
    YoubotDynamics* youbot_dynamics_;

    CommandMode command_mode_;
    luh_youbot_kinematics::JointVelocity last_velocity_;
    ros::Time last_update_time_;

    void update();
    void activate();
    void deactivate();
    void init();
    void emergencyStop();

    bool load(std::string posefile, std::string neighborfile);
    void resetPoses();
    void updateNeighborCosts(Pose* current_pose, luh_youbot_kinematics::JointPosition end_position);
    void addNeighbors(PoseSet &open, Pose* current_pose);

    double costFunction(const luh_youbot_kinematics::JointPosition &position, const Pose &pose);
    double costFunction(const Pose &pos1, const Pose &pos2);

    bool predefPoseCallback(luh_youbot_msgs::ToPredefinedPose::Request &req,
                            luh_youbot_msgs::ToPredefinedPose::Response &res);
    bool listPosesCallback(luh_youbot_msgs::ListPoses::Request &req, luh_youbot_msgs::ListPoses::Response &res);

    void jointPoseCallback();
    void cartesianPoseCallback();
    void cylindricPoseCallback();
    void namedPoseCallback();
    void jointPathCallback();
    void cartesianPathCallback();
    void cylindricPathCallback();
    void namedPathCallback();

    void preemptCallback();

    void endJointPoseAction(bool success);
    void endCartesianPoseAction(bool success);
    void endCylindricPoseAction(bool success);
    void endNamedPoseAction(bool success);
    void endJointPathAction(bool success);
    void endCartesianPathAction(bool success);
    void endCylindricPathAction(bool success);
    void endNamedPathAction(bool success);
    void endPredefinedPoseService(bool success);

    bool isReady();

    void startMovement(const Path &path);

    void (ModuleMotionPlanner::*endAction)(bool);

    void endMovement();

    bool goalReached();

};

#endif // MOTIONPLANNER_H
