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

#include "luh_youbot_controller/node/controller_node.h"
#include "luh_youbot_controller/module_interpolation/module_interpolation.h"
#include "luh_youbot_controller/module_motion_planner/motion_planner.h"
#include "luh_youbot_controller/module_base_controller/base_controller.h"
#include "luh_youbot_controller/module_gripper/gripper.h"
#include "luh_youbot_controller/module_direct_control/module_direct_control.h"
#include "luh_youbot_controller/module_joint_trajectory/module_joint_trajectory.h"
#include "luh_youbot_controller/module_gravity_compensation/module_gravity_compensation.h"
#include <luh_youbot_vrep_api/youbot_interface.h>
#include <luh_youbot_gazebo/youbot_interface.h>

using namespace luh_youbot_kinematics;

//########################## CONSTRUCTOR ###############################################################################
ControllerNode::ControllerNode(ros::NodeHandle &node):
    node_(&node)
{
    // === PARAMETERS ===
    node_->param("luh_youbot_controller/arm_controller_frequency", arm_frequency_, 200.0);
    node_->param("luh_youbot_controller/base_controller_frequency", base_frequency_, 50.0);
    node_->param("luh_youbot_controller/use_standard_gripper", use_standard_gripper_, true);
    node_->param("luh_youbot_controller/use_vrep_simulation", use_vrep_simulation_, false);
    node_->param("luh_youbot_controller/use_gazebo_simulation", use_gazebo_simulation_, false);
    node_->param("luh_youbot_controller/use_luh_gripper_v3", use_luh_gripper_v3_, false);

    if(use_standard_gripper_)
        ROS_INFO("Using standard gripper.");
    else if(use_luh_gripper_v3_)
        ROS_INFO("Using LUH-Gripper Version 3.0");
    else
        ROS_INFO("Using external gripper.");


    // === YOUBOT INTERFACE ===
    if(use_gazebo_simulation_)
    {
        ROS_WARN("Running in GAZEBO simulation mode.");
        youbot_ = new YoubotGazeboInterface(node);
    }
    else if(use_vrep_simulation_)
    {
        ROS_WARN("Running in VREP simulation mode.");
        youbot_ = new YoubotVrepInterface(node);
    }    
    else
        youbot_ = new YoubotInterface(node);

    youbot_->initialise(use_standard_gripper_,use_luh_gripper_v3_);

    // === TIMER ===
    arm_timer_ = node_->createTimer(ros::Duration(1.0/arm_frequency_),
                                    &ControllerNode::armTimerCallback, this, false, false);
    base_timer_ = node_->createTimer(ros::Duration(1.0/base_frequency_),
                                     &ControllerNode::baseTimerCallback, this, false, false);

    // === SERVICE SERVERS ===
    get_pose_server_ =
            node_->advertiseService("arm_1/get_pose", &ControllerNode::getPoseCallback, this);
    stop_arm_server_ =
            node_->advertiseService("arm_1/stop", &ControllerNode::stopArmCallback, this);
    stop_base_server_ =
            node_->advertiseService("base/stop", &ControllerNode::stopBaseCallback, this);
    stop_bot_server_ =
            node_->advertiseService("youbot/stop", &ControllerNode::stopBotCallback, this);
    enable_ramp_server_ =
            node_->advertiseService("arm_1/enable_ramp", &ControllerNode::enableRampCallback, this);
    disable_ramp_server_ =
            node_->advertiseService("arm_1/disable_ramp", &ControllerNode::disableRampCallback, this);

    // === INIT MODULES ===
    ControllerModule::setArmIsBusy(false);
    ControllerModule::setBaseIsBusy(false);
    ControllerModule::setGripperIsBusy(false);
    ControllerModule::initStatic(node_, tf_listener_, youbot_);
    ControllerModule::setArmUpdateFrequency(arm_frequency_);
    ControllerModule::setBaseUpdateFrequency(base_frequency_);

    // === INIT ARM ===
    if(youbot_->hasArms())
    {
        arm_modules_.push_back(new ModuleInterpolation());
        arm_modules_.push_back(new ModuleMotionPlanner());        
        arm_modules_.push_back(new ModuleDirectControl());
        arm_modules_.push_back(new ModuleJointTrajectory());
        arm_modules_.push_back(new ModuleGravityCompensation());

        if(use_standard_gripper_ || use_luh_gripper_v3_)
            arm_modules_.push_back(new ModuleGripper());

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
        ControllerModule::setArmIsBusy(true);
        ControllerModule::setGripperIsBusy(true);
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
        ControllerModule::setBaseIsBusy(true);

    ROS_INFO("All modules initialised.");
}

//########################## DESTRUCTOR ################################################################################
ControllerNode::~ControllerNode()
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
void ControllerNode::armTimerCallback(const ros::TimerEvent &evt)
{
    boost::mutex::scoped_lock armlock(ControllerModule::arm_mutex_);
    boost::mutex::scoped_lock gripperlock(ControllerModule::gripper_mutex_);

    if(!youbot_->arm()->isInitialised())
        return;

    // === READ ARM SENSORS ===
    ethercat_mutex_.lock();
    youbot_->arm()->readState();
    ethercat_mutex_.unlock();

    // === UPDATE MODULES ===
    for(uint i=0; i<arm_modules_.size(); i++)
    {
        arm_modules_[i]->update();
    }

    // === SECURITY CHECK ===
    int error_joint = youbot_->arm()->securityCheck();
    if(error_joint > 0)
    {
        ROS_INFO("Overcurrent in joint %d", error_joint);
        stopArm();
    }

    // === WRITE ARM COMMANDS ===
    else
    {
        ethercat_mutex_.lock();
        bool error = !youbot_->arm()->writeCommands();
        ethercat_mutex_.unlock();

        if(error)
            stopArm();
    }

    // == PUBLISH JOINT STATE MESSAGES ===
    youbot_->arm()->publishMessages();

    // === CHECK FREQUENCY ===
    double time_delay = (evt.current_real - evt.current_expected).toSec();
    if(fabs(time_delay) > 2.0 / arm_frequency_)
    {
        ROS_WARN("Loop is slower than desired frequency of %f Hz.", arm_frequency_);
        ROS_WARN("Current delay is: %f", time_delay);
    }
}

//########################## CALLBACK: BASE TIMER ######################################################################
void ControllerNode::baseTimerCallback(const ros::TimerEvent &evt)
{
    boost::mutex::scoped_lock lock(ControllerModule::base_mutex_);

    if(!youbot_->base()->isInitialised())
        return;

    // === READ BASE SENSORS ===
    ethercat_mutex_.lock();
    youbot_->base()->readState();
    ethercat_mutex_.unlock();

    // === UPDATE MODULES ===
    for(uint i=0; i<base_modules_.size(); i++)
    {
        base_modules_[i]->update();
    }

    // === UPDATE BASE CONTROLLER ===
    youbot_->base()->updateController();

    // === WRITE BASE COMMANDS ===
    ethercat_mutex_.lock();
    bool error = !youbot_->base()->writeCommands();
    ethercat_mutex_.unlock();

    if(error)
        stopBase();

    // == PUBLISH BASE MESSAGES ===
    youbot_->base()->publishMessages();
}

//########################## CALLBACK: GET POSE ########################################################################
bool ControllerNode::getPoseCallback(luh_youbot_msgs::GetArmPose::Request &req,
                                  luh_youbot_msgs::GetArmPose::Response &res)
{
    boost::mutex::scoped_lock lock(ControllerModule::arm_mutex_);

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
void ControllerNode::stopArm()
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
void ControllerNode::stopBase()
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
bool ControllerNode::stopArmCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock lock(ControllerModule::arm_mutex_);

    stopArm();
    return true;
}

//########################## CALLBACK: EMERGENCY STOP BASE #############################################################
bool ControllerNode::stopBaseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock lock(ControllerModule::base_mutex_);

    stopBase();
    return true;
}

//########################## CALLBACK: EMERGENCY STOP BOT ##############################################################
bool ControllerNode::stopBotCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock armlock(ControllerModule::arm_mutex_);
    boost::mutex::scoped_lock baselock(ControllerModule::base_mutex_);

    stopArm();
    stopBase();

    return true;
}

//########################## CALLBACK: ENABLE RAMP #####################################################################
bool ControllerNode::enableRampCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock lock(ControllerModule::arm_mutex_);
    youbot_->arm()->enableRampGenerator(true);

    return true;
}

//########################## CALLBACK: DISABLE RAMP ####################################################################
bool ControllerNode::disableRampCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock lock(ControllerModule::arm_mutex_);
    youbot_->arm()->enableRampGenerator(false);

    return true;
}
