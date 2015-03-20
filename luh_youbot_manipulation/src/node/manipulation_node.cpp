/* *****************************************************************
 *
 * luh_youbot_manipulation
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
 * Author: Simon Aden (simon.aden@mailbox.org)
 ******************************************************************/

#include "luh_youbot_manipulation/node/manipulation_node.h"
#include "luh_youbot_manipulation/module_interpolation/module_interpolation.h"
#include "luh_youbot_manipulation/module_motion_planner/motion_planner.h"
#include "luh_youbot_manipulation/module_base_controller/base_controller.h"
#include "luh_youbot_manipulation/module_gripper/gripper.h"
#include "luh_youbot_manipulation/module_direct_control/module_direct_control.h"
#include "luh_youbot_manipulation/module_joint_trajectory/module_joint_trajectory.h"
#include "luh_youbot_manipulation/module_gravity_compensation/module_gravity_compensation.h"

using namespace luh_youbot_kinematics;

//########################## CONSTRUCTOR ###############################################################################
ManipulationNode::ManipulationNode(ros::NodeHandle &node):
    node_(&node)
{
    // === PARAMETERS ===
    node_->param("luh_youbot_manipulation/arm_controller_frequency", arm_frequency_, 200.0);
    node_->param("luh_youbot_manipulation/base_controller_frequency", base_frequency_, 50.0);

    // === YOUBOT INTERFACE ===
    youbot_ = new YoubotInterface(node);
    youbot_->initialise();

    // === TIMER ===
    arm_timer_ = node_->createTimer(ros::Duration(1.0/arm_frequency_),
                                    &ManipulationNode::armTimerCallback, this, false, false);
    base_timer_ = node_->createTimer(ros::Duration(1.0/base_frequency_),
                                     &ManipulationNode::baseTimerCallback, this, false, false);

    // === SERVICE SERVERS ===
    get_pose_server_ =
            node_->advertiseService("arm_1/get_pose", &ManipulationNode::getPoseCallback, this);
    stop_arm_server_ =
            node_->advertiseService("arm_1/stop", &ManipulationNode::stopArmCallback, this);
    stop_base_server_ =
            node_->advertiseService("base/stop", &ManipulationNode::stopBaseCallback, this);
    stop_bot_server_ =
            node_->advertiseService("youbot/stop", &ManipulationNode::stopBotCallback, this);

    // === INIT MODULES ===
    ManipulationModule::setArmIsBusy(false);
    ManipulationModule::setBaseIsBusy(false);
    ManipulationModule::setGripperIsBusy(false);
    ManipulationModule::initStatic(node_, tf_listener_, youbot_);
    ManipulationModule::setArmUpdateFrequency(arm_frequency_);
    ManipulationModule::setBaseUpdateFrequency(base_frequency_);

    // === INIT ARM ===
    if(youbot_->hasArms())
    {
        arm_modules_.push_back(new ModuleInterpolation());
        arm_modules_.push_back(new ModuleMotionPlanner());
        arm_modules_.push_back(new ModuleGripper());
        arm_modules_.push_back(new ModuleDirectControl());
        arm_modules_.push_back(new ModuleJointTrajectory());
        arm_modules_.push_back(new ModuleGravityCompensation());
        // other modules can be added here

        for(uint i=0; i<arm_modules_.size(); i++)
        {
            arm_modules_[i]->init();
        }

        ROS_INFO("Arm timer started with %f Hz.", arm_frequency_);
        arm_timer_.start();
    }
    else
    {
        ManipulationModule::setArmIsBusy(true);
        ManipulationModule::setGripperIsBusy(true);
    }


    // === INIT BASE ===
    if(youbot_->hasBase())
    {
        base_modules_.push_back(new ModuleBaseController());
        // other modules can be added here

        for(uint i=0; i<base_modules_.size(); i++)
        {
            base_modules_[i]->init();
        }

        ROS_INFO("Base timer started with %f Hz.", base_frequency_);
        base_timer_.start();
    }
    else
        ManipulationModule::setBaseIsBusy(true);

    ROS_INFO("All modules initialised.");
}

//########################## DESTRUCTOR ################################################################################
ManipulationNode::~ManipulationNode()
{
    for(uint i=0; i<arm_modules_.size(); i++)
    {
        delete arm_modules_[i];
    }

    for(uint i=0; i<base_modules_.size(); i++)
    {
        delete base_modules_[i];
    }

    youbot_->stop();

    delete youbot_;
}

//########################## CALLBACK: ARM TIMER #######################################################################
void ManipulationNode::armTimerCallback(const ros::TimerEvent &evt)
{
    if(!youbot_->arm()->isInitialised())
        return;

    // === READ ARM SENSORS ===
    youbot_->arm()->readState();

    // === UPDATE MODULES ===
    for(uint i=0; i<arm_modules_.size(); i++)
    {
        arm_modules_[i]->update();
    }

    // === SECURITY CHECK ===
    if(!youbot_->arm()->securityCheck())
        stopArm();

    // === WRITE ARM COMMANDS ===
    else if(!youbot_->arm()->writeCommands())
        stopArm();

    // == PUBLISH JOINT STATE MESSAGES ===
    youbot_->arm()->publishMessages();
}

//########################## CALLBACK: BASE TIMER ######################################################################
void ManipulationNode::baseTimerCallback(const ros::TimerEvent &evt)
{
    if(!youbot_->base()->isInitialised())
        return;

    // === READ BASE SENSORS ===
    youbot_->base()->readState();

    // === UPDATE MODULES ===
    for(uint i=0; i<base_modules_.size(); i++)
    {
        base_modules_[i]->update();
    }

    // === UPDATE BASE CONTROLLER ===
    youbot_->base()->updateController();

    // === WRITE BASE COMMANDS ===
    if(!youbot_->base()->writeCommands())
        stopBase();

    // == PUBLISH BASE MESSAGES ===
    youbot_->base()->publishMessages();
}

//########################## CALLBACK: GET POSE ########################################################################
bool ManipulationNode::getPoseCallback(luh_youbot_msgs::GetArmPose::Request &req,
                                  luh_youbot_msgs::GetArmPose::Response &res)
{
    JointPosition current_position = youbot_->arm()->getJointPosition();

    res.joint_pose.q1 = current_position.q1();
    res.joint_pose.q2 = current_position.q2();
    res.joint_pose.q3 = current_position.q3();
    res.joint_pose.q4 = current_position.q4();
    res.joint_pose.q5 = current_position.q5();

    CartesianPosition cart_pos = youbot_->arm()->getJointPosition().toCartesian();
    res.cartesian_pose.x = cart_pos.x();
    res.cartesian_pose.y = cart_pos.y();
    res.cartesian_pose.z = cart_pos.z();
    res.cartesian_pose.theta = cart_pos.theta();
    res.cartesian_pose.q5 = cart_pos.q5();
    res.cartesian_pose.header.frame_id = "arm_link_0";


    CylindricPosition cyl_pos = youbot_->arm()->getJointPosition().toCylindric();
    res.cylindric_pose.r = cyl_pos.r();
    res.cylindric_pose.q1 = cyl_pos.q1();
    res.cylindric_pose.z = cyl_pos.z();
    res.cylindric_pose.theta = cyl_pos.theta();
    res.cylindric_pose.q5 = cyl_pos.q5();
    res.cylindric_pose.header.frame_id = "cylindric";


    return true;
}

//########################## STOP ARM ##################################################################################
void ManipulationNode::stopArm()
{
    if(!youbot_->hasArms())
        return;

    ROS_INFO("Stopping all arm modules.");
    for(uint i=0; i<arm_modules_.size(); i++)
    {
        arm_modules_[i]->emergencyStop();        
    }
}

//########################## STOP BASE #################################################################################
void ManipulationNode::stopBase()
{
    if(!youbot_->hasBase())
        return;

    ROS_INFO("Stopping all base modules.");
    for(uint i=0; i<base_modules_.size(); i++)
    {
        base_modules_[i]->emergencyStop();
    }
}

//########################## CALLBACK: EMERGENCY STOP ARM ##############################################################
bool ManipulationNode::stopArmCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    stopArm();
    return true;
}

//########################## CALLBACK: EMERGENCY STOP BASE #############################################################
bool ManipulationNode::stopBaseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    stopBase();
    return true;
}

//########################## CALLBACK: EMERGENCY STOP BOT ##############################################################
bool ManipulationNode::stopBotCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    stopArm();
    stopBase();

    return true;
}
