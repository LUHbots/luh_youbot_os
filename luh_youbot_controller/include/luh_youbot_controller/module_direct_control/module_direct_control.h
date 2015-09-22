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

#ifndef LUH_YOUBOT_CONTROLLER_MODULE_DIRECT_CONTROL_H
#define LUH_YOUBOT_CONTROLLER_MODULE_DIRECT_CONTROL_H

#include "../module_base_class/controller_module.h"
#include "actionlib/server/simple_action_server.h"
#include "luh_youbot_msgs/MoveToCartesianPoseAction.h"
#include "luh_youbot_msgs/MoveToCylindricPoseAction.h"
#include "luh_youbot_msgs/MoveToJointPoseAction.h"
#include "luh_youbot_msgs/MoveToNamedPoseAction.h"
#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToCartesianPoseAction> CartesianServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToCylindricPoseAction> CylindricServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToJointPoseAction> JointServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveToNamedPoseAction> NamedServer;


class ModuleDirectControl : public ControllerModule
{
public:
    ModuleDirectControl();
    ~ModuleDirectControl();

    void init();
    void activate();
    void deactivate();
    void update();
    void emergencyStop();

protected:

    // parameters
    double joint_position_tolerance_;
    double joint_velocity_tolerance_;

    // action servers
    CartesianServer* cartesian_pose_server_;
    CylindricServer* cylindric_pose_server_;
    JointServer* joint_pose_server_;
    NamedServer* named_pose_server_;

    // subscriber
    ros::Subscriber joint_position_subscriber_;

    // service servers
    ros::ServiceServer relax_server_;
    ros::ServiceServer stiffen_server_;

    // callbacks
    void cartesianPoseCallback();
    void cylindricPoseCallback();
    void jointPoseCallback();
    void namedPoseCallback();
    void preemptCallback();
    bool relaxCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool stiffenCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void jointPositionCallback(const luh_youbot_msgs::JointVector::ConstPtr pos);

    void preempt();

    bool active_;
    bool has_command_;
    luh_youbot_kinematics::JointPosition position_command_;
};

#endif // MODULE_DIRECT_CONTROL_H
