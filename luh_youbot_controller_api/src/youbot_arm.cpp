/* *****************************************************************
 *
 * luh_youbot_controller_api
 *
 * Copyright (c) 2014,
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
 ******************************************************************/

/**
 * \file
 * @author Simon Aden (info@luhbots.de)
 * @date   November 2014
 *
 * @brief  Source file for the YoubotArm class.
 */

#include "luh_youbot_controller_api/youbot_arm.h"
#include <luh_youbot_msgs/interpolation_mode.h>
#include <luh_youbot_msgs/SetCartesianVelocity.h>
#include <luh_youbot_msgs/SetCylindricVelocity.h>
#include <luh_youbot_msgs/SetJointVelocity.h>

namespace youbot_api
{

//########## CONSTRUCTOR ###############################################################################################
YoubotArm::YoubotArm():
    arm_is_busy_(false),
    cart_client_direct_(NULL),
    cart_client_inter_(NULL),
    cart_client_plan_(NULL),
    cart_traj_client_(NULL),
    cyl_client_direct_(NULL),
    cyl_client_inter_(NULL),
    cyl_client_plan_(NULL),
    cyl_traj_client_(NULL),
    jnt_client_direct_(NULL),
    jnt_client_inter_(NULL),
    jnt_client_plan_(NULL),
    jnt_traj_client_(NULL),
    name_client_direct_(NULL),
    name_client_inter_(NULL),
    name_client_plan_(NULL),
    name_traj_client_(NULL)
{
    // enable all clients by default
    is_enabled_[ActionClient::CART_DIRECT] = true;
    is_enabled_[ActionClient::CART_INTER] = true;
    is_enabled_[ActionClient::CART_PLAN] = true;
    is_enabled_[ActionClient::CART_PATH] = true;
    is_enabled_[ActionClient::CYL_DIRECT] = true;
    is_enabled_[ActionClient::CYL_INTER] = true;
    is_enabled_[ActionClient::CYL_PLAN] = true;
    is_enabled_[ActionClient::CYL_PATH] = true;
    is_enabled_[ActionClient::JOINT_DIRECT] = true;
    is_enabled_[ActionClient::JOINT_INTER] = true;
    is_enabled_[ActionClient::JOINT_PLAN] = true;
    is_enabled_[ActionClient::JOINT_PATH] = true;
    is_enabled_[ActionClient::NAME_DIRECT] = true;
    is_enabled_[ActionClient:: NAME_INTER] = true;
    is_enabled_[ActionClient::NAME_PLAN] = true;
    is_enabled_[ActionClient::NAME_PATH] = true;
}

//########## DESTRUCTOR ################################################################################################
YoubotArm::~YoubotArm()
{
    delete cart_client_direct_;
    delete cart_client_inter_;
    delete cart_client_plan_;
    delete cart_traj_client_;

    delete cyl_client_direct_;
    delete cyl_client_inter_;
    delete cyl_client_plan_;
    delete cyl_traj_client_;

    delete jnt_client_direct_;
    delete jnt_client_inter_;
    delete jnt_client_plan_;
    delete jnt_traj_client_;

    delete name_client_direct_;
    delete name_client_inter_;
    delete name_client_plan_;
    delete name_traj_client_;
}

//########## INIT ######################################################################################################
void YoubotArm::init(ros::NodeHandle &node)
{
    // save pointer to node handle
    node_ = &node;

    ROS_INFO("Waiting for arm action servers...");

    // create action clients
    if(is_enabled_[ActionClient::CART_DIRECT])
    {
        cart_client_direct_  = new CartesianPoseClient("arm_1/to_cartesian_pose/direct", true);
        cart_client_direct_->waitForServer();
    }
    if(is_enabled_[ActionClient::CART_INTER])
    {
        cart_client_inter_   = new CartesianPoseClient("arm_1/to_cartesian_pose/inter", true);
        cart_client_inter_->waitForServer();
    }
    if(is_enabled_[ActionClient::CART_PLAN])
    {
        cart_client_plan_ = new CartesianPoseClient("arm_1/to_cartesian_pose/plan", true);
        cart_client_plan_->waitForServer();
    }
    if(is_enabled_[ActionClient::CYL_DIRECT])
    {
        cyl_client_direct_  = new CylindricPoseClient("arm_1/to_cylindric_pose/direct", true);
        cyl_client_direct_->waitForServer();
    }
    if(is_enabled_[ActionClient::CYL_INTER])
    {
        cyl_client_inter_   = new CylindricPoseClient("arm_1/to_cylindric_pose/inter", true);
        cyl_client_inter_->waitForServer();
    }
    if(is_enabled_[ActionClient::CYL_PLAN])
    {
        cyl_client_plan_ = new CylindricPoseClient("arm_1/to_cylindric_pose/plan", true);
        cyl_client_plan_->waitForServer();
    }
    if(is_enabled_[ActionClient::JOINT_DIRECT])
    {
        jnt_client_direct_  = new JointPoseClient("arm_1/to_joint_pose/direct", true);
        jnt_client_direct_->waitForServer();
    }
    if(is_enabled_[ActionClient::JOINT_INTER])
    {
        jnt_client_inter_   = new JointPoseClient("arm_1/to_joint_pose/inter", true);
        jnt_client_inter_->waitForServer();
    }
    if(is_enabled_[ActionClient::JOINT_PLAN])
    {
        jnt_client_plan_ = new JointPoseClient("arm_1/to_joint_pose/plan", true);
        jnt_client_plan_->waitForServer();
    }
    if(is_enabled_[ActionClient::NAME_DIRECT])
    {
        name_client_direct_  = new NamedPoseClient("arm_1/to_named_pose/direct", true);
        name_client_direct_->waitForServer();
    }
    if(is_enabled_[ActionClient::NAME_INTER])
    {
        name_client_inter_   = new NamedPoseClient("arm_1/to_named_pose/inter", true);
        name_client_inter_->waitForServer();
    }
    if(is_enabled_[ActionClient::NAME_PLAN])
    {
        name_client_plan_ = new NamedPoseClient("arm_1/to_named_pose/plan", true);
        name_client_plan_->waitForServer();
    }
    if(is_enabled_[ActionClient::CART_PATH])
    {
        cart_traj_client_ = new CartesianPathClient("arm_1/cartesian_path", true);
        cart_traj_client_->waitForServer();
    }
    if(is_enabled_[ActionClient::CYL_PATH])
    {
        cyl_traj_client_ = new CylindricPathClient("arm_1/cylindric_path", true);
        cyl_traj_client_->waitForServer();
    }
    if(is_enabled_[ActionClient::JOINT_PATH])
    {
        jnt_traj_client_ = new JointPathClient("arm_1/joint_path", true);
        jnt_traj_client_->waitForServer();
    }
    if(is_enabled_[ActionClient::NAME_PATH])
    {
        name_traj_client_ = new NamedPathClient("arm_1/named_path", true);
        name_traj_client_->waitForServer();
    }

    set_cart_vel_client_ = node.serviceClient<luh_youbot_msgs::SetCartesianVelocity>("arm_1/set_cartesian_velocity");
    set_cyl_vel_client_ = node.serviceClient<luh_youbot_msgs::SetCylindricVelocity>("arm_1/set_cylindirc_velocity");
    set_jnt_vel_client_ = node.serviceClient<luh_youbot_msgs::SetJointVelocity>("arm_1/set_joint_velocity");

    ROS_INFO("Arm action clients initialised.");
}

//########## MOVE TO POSE (JOINTSPACE) #################################################################################
void YoubotArm::moveToPose(luh_youbot_kinematics::JointPosition pose, int mode, bool pose_is_relative)
{
    luh_youbot_msgs::MoveToJointPoseGoal goal;
    goal.pose_is_relative = pose_is_relative;
    goal.pose.q1 = pose.q1();
    goal.pose.q2 = pose.q2();
    goal.pose.q3 = pose.q3();
    goal.pose.q4 = pose.q4();
    goal.pose.q5 = pose.q5();

    if(mode == MotionMode::DIRECT)
    {
        active_client_ = ActionClient::JOINT_DIRECT;
        arm_is_busy_ = true;
        jnt_client_direct_->sendGoal(goal, boost::bind(&YoubotArm::onJointActionDone, this, _1, _2));
    }
    else if(mode == MotionMode::INTERPOLATE)
    {
        active_client_ = ActionClient::JOINT_INTER;
        arm_is_busy_ = true;
        jnt_client_inter_->sendGoal(goal, boost::bind(&YoubotArm::onJointActionDone, this, _1, _2));
    }
    else if(mode == MotionMode::PLAN)
    {
        active_client_ = ActionClient::JOINT_PLAN;
        arm_is_busy_ = true;
        jnt_client_plan_->sendGoal(goal, boost::bind(&YoubotArm::onJointActionDone, this, _1, _2));
    }
}

//########## MOVE TO POSE (CYLINDRIC) ##################################################################################
void YoubotArm::moveToPose(luh_youbot_kinematics::CylindricPosition pose, int mode, bool pose_is_relative)
{
    luh_youbot_msgs::MoveToCylindricPoseGoal goal;
    goal.pose_is_relative = pose_is_relative;
    goal.pose.q1 = pose.q1();
    goal.pose.r = pose.r();
    goal.pose.z = pose.z();
    goal.pose.theta = pose.theta();
    goal.pose.q5 = pose.q5();

    if(mode == MotionMode::DIRECT)
    {
        active_client_ = ActionClient::CYL_DIRECT;
        arm_is_busy_ = true;
        cyl_client_direct_->sendGoal(goal, boost::bind(&YoubotArm::onCylActionDone, this, _1, _2));
    }
    else if(mode == MotionMode::INTERPOLATE)
    {
        active_client_ = ActionClient::CYL_INTER;
        arm_is_busy_ = true;
        cyl_client_inter_->sendGoal(goal, boost::bind(&YoubotArm::onCylActionDone, this, _1, _2));
    }
    else if(mode == MotionMode::PLAN)
    {
        active_client_ = ActionClient::CYL_PLAN;
        arm_is_busy_ = true;
        cyl_client_plan_->sendGoal(goal, boost::bind(&YoubotArm::onCylActionDone, this, _1, _2));
    }
}

//########## MOVE TO POSE (CARTESIAN) ##################################################################################
void YoubotArm::moveToPose(luh_youbot_kinematics::CartesianPosition pose, int mode, std::string frame_id,
                           bool pose_is_relative)
{
    luh_youbot_msgs::MoveToCartesianPoseGoal goal;
    goal.pose_is_relative = pose_is_relative;
    goal.pose.header.frame_id = frame_id;
    goal.pose.x = pose.x();
    goal.pose.y = pose.y();
    goal.pose.z = pose.z();
    goal.pose.theta = pose.theta();
    goal.pose.q5 = pose.q5();

    if(mode == MotionMode::DIRECT)
    {
        active_client_ = ActionClient::CART_DIRECT;
        arm_is_busy_ = true;
        cart_client_direct_->sendGoal(goal, boost::bind(&YoubotArm::onCartActionDone, this, _1, _2));
    }
    else if(mode == MotionMode::INTERPOLATE)
    {
        active_client_ = ActionClient::CART_INTER;
        arm_is_busy_ = true;
        cart_client_inter_->sendGoal(goal, boost::bind(&YoubotArm::onCartActionDone, this, _1, _2));
    }
    else if(mode == MotionMode::PLAN)
    {
        active_client_ = ActionClient::CART_PLAN;
        arm_is_busy_ = true;
        cart_client_plan_->sendGoal(goal, boost::bind(&YoubotArm::onCartActionDone, this, _1, _2));
    }
}

//########## MOVE TO POSE (NAMED) ######################################################################################
void YoubotArm::moveToPose(std::string pose_name, int movement_mode, int interpolation_mode)
{
    luh_youbot_msgs::MoveToNamedPoseGoal goal;
    goal.pose_name = pose_name;

    if(interpolation_mode == InterpolationMode::JOINTSPACE)
        goal.interpolation_mode = luh_youbot_msgs::JOINTSPACE;

    else if(interpolation_mode == InterpolationMode::CARTESIAN)
        goal.interpolation_mode = luh_youbot_msgs::CARTESIAN;

    else if(interpolation_mode == InterpolationMode::CYLINDRIC)
        goal.interpolation_mode = luh_youbot_msgs::CYLINDRIC;

    if(movement_mode == MotionMode::DIRECT)
    {
        active_client_ = ActionClient::NAME_DIRECT;
        arm_is_busy_ = true;
        name_client_direct_->sendGoal(goal, boost::bind(&YoubotArm::onNamedActionDone, this, _1, _2));
    }
    else if(movement_mode == MotionMode::INTERPOLATE)
    {
        active_client_ = ActionClient::NAME_INTER;
        arm_is_busy_ = true;
        name_client_inter_->sendGoal(goal, boost::bind(&YoubotArm::onNamedActionDone, this, _1, _2));
    }
    else if(movement_mode == MotionMode::PLAN)
    {
        active_client_ = ActionClient::NAME_PLAN;
        arm_is_busy_ = true;
        name_client_plan_->sendGoal(goal, boost::bind(&YoubotArm::onNamedActionDone, this, _1, _2));
    }
}

//########## MOVE ALONG JOINT PATHECTORY ###############################################################################
void YoubotArm::moveAlongPath(JointPath path)
{
    luh_youbot_msgs::MoveJointPathGoal goal;
    goal.path.poses.resize(path.size());

    for(uint i=0; i<path.size(); i++)
    {
        goal.path.poses[i].q1 = path[i].q1();
        goal.path.poses[i].q2 = path[i].q2();
        goal.path.poses[i].q3 = path[i].q3();
        goal.path.poses[i].q4 = path[i].q4();
        goal.path.poses[i].q5 = path[i].q5();
    }

    active_client_ = ActionClient::JOINT_PATH;
    arm_is_busy_ = true;
    jnt_traj_client_->sendGoal(goal, boost::bind(&YoubotArm::onJointTrajActionDone, this, _1, _2));
}

//########## MOVE ALONG CYLINDRIC PATHECTORY ###########################################################################
void YoubotArm::moveAlongPath(CylindricPath path)
{
    luh_youbot_msgs::MoveCylindricPathGoal goal;
    goal.path.poses.resize(path.size());

    for(uint i=0; i<path.size(); i++)
    {
        goal.path.poses[i].q1 = path[i].q1();
        goal.path.poses[i].r = path[i].r();
        goal.path.poses[i].z = path[i].z();
        goal.path.poses[i].theta = path[i].theta();
        goal.path.poses[i].q5 = path[i].q5();
    }

    active_client_ = ActionClient::CYL_PATH;
    arm_is_busy_ = true;
    cyl_traj_client_->sendGoal(goal, boost::bind(&YoubotArm::onCylTrajActionDone, this, _1, _2));
}

//########## MOVE ALONG CARTESIAN PATHECTORY ###########################################################################
void YoubotArm::moveAlongPath(CartesianPath path)
{
    luh_youbot_msgs::MoveCartesianPathGoal goal;
    goal.path.poses.resize(path.size());

    for(uint i=0; i<path.size(); i++)
    {
        goal.path.poses[i].x = path[i].x();
        goal.path.poses[i].y = path[i].y();
        goal.path.poses[i].z = path[i].z();
        goal.path.poses[i].theta = path[i].theta();
        goal.path.poses[i].q5 = path[i].q5();
    }

    active_client_ = ActionClient::CART_PATH;
    arm_is_busy_ = true;
    cart_traj_client_->sendGoal(goal, boost::bind(&YoubotArm::onCartTrajActionDone, this, _1, _2));
}

//########## MOVE ALONG NAMED PATHECTORY ###############################################################################
void YoubotArm::moveAlongPath(NamedPath path)
{
    luh_youbot_msgs::MoveNamedPathGoal goal;

    for(uint i=0; i<path.size(); i++)
    {
        goal.path.poses.push_back(path[i]);
    }

    active_client_ = ActionClient::NAME_PATH;
    arm_is_busy_ = true;
    name_traj_client_->sendGoal(goal, boost::bind(&YoubotArm::onNamedTrajActionDone, this, _1, _2));
}

//########## WAIT FOR ARM ##############################################################################################
bool YoubotArm::waitForCurrentAction(double timeout)
{
    if(!arm_is_busy_)
        return true;

    bool done = false;
    switch(active_client_)
    {
    case(ActionClient::CART_DIRECT):
    {
        done = cart_client_direct_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::CART_INTER):
    {
        done = cart_client_inter_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::CART_PLAN):
    {
        done = cart_client_plan_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::CART_PATH):
    {
        done = cart_traj_client_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::CYL_DIRECT):
    {
        done = cyl_client_direct_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::CYL_INTER):
    {
        done = cyl_client_inter_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::CYL_PLAN):
    {
        done = cyl_client_plan_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::CYL_PATH):
    {
        done = cyl_traj_client_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::JOINT_DIRECT):
    {
        done = jnt_client_direct_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::JOINT_INTER):
    {
        done = jnt_client_inter_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::JOINT_PLAN):
    {
        done = jnt_client_plan_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::JOINT_PATH):
    {
        done = jnt_traj_client_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::NAME_DIRECT):
    {
        done = name_client_direct_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::NAME_INTER):
    {
        done = name_client_inter_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::NAME_PLAN):
    {
        done = name_client_plan_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(ActionClient::NAME_PATH):
    {
        done = name_traj_client_->waitForResult(ros::Duration(timeout));
        break;
    }
    }

    if(done)
    {
        arm_is_busy_ = false;
    }

    return done;
}

//########## ON JOINT ACTION DONE ######################################################################################
void YoubotArm::onJointActionDone(const actionlib::SimpleClientGoalState& state,
                                  const luh_youbot_msgs::MoveToJointPoseResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ON CARTESIAN ACTION DONE ##################################################################################
void YoubotArm::onCartActionDone(const actionlib::SimpleClientGoalState& state,
                                 const luh_youbot_msgs::MoveToCartesianPoseResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ON CYLINDRIC ACTION DONE ##################################################################################
void YoubotArm::onCylActionDone(const actionlib::SimpleClientGoalState& state,
                                const luh_youbot_msgs::MoveToCylindricPoseResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ON NAMED ACTION DONE ######################################################################################
void YoubotArm::onNamedActionDone(const actionlib::SimpleClientGoalState& state,
                                  const luh_youbot_msgs::MoveToNamedPoseResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ON JOINT PATHECTORY ACTION DONE ###########################################################################
void YoubotArm::onJointTrajActionDone(const actionlib::SimpleClientGoalState& state,
                                      const luh_youbot_msgs::MoveJointPathResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ON CARTESIAN PATHECTORY ACTION DONE #######################################################################
void YoubotArm::onCartTrajActionDone(const actionlib::SimpleClientGoalState& state,
                                     const luh_youbot_msgs::MoveCartesianPathResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ON CYLINDRIC PATHECTORY ACTION DONE #######################################################################
void YoubotArm::onCylTrajActionDone(const actionlib::SimpleClientGoalState& state,
                                    const luh_youbot_msgs::MoveCylindricPathResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ON NAMED PATHECTORY ACTION DONE ###########################################################################
void YoubotArm::onNamedTrajActionDone(const actionlib::SimpleClientGoalState& state,
                                      const luh_youbot_msgs::MoveNamedPathResultConstPtr& result)
{
    arm_is_busy_ = false;
}

//########## ABORT CURRENT ACTION ######################################################################################
void YoubotArm::abortCurrentAction()
{
    switch(active_client_)
    {
    case(ActionClient::CART_DIRECT):
    {
        cart_client_direct_->cancelAllGoals();
        break;
    }
    case(ActionClient::CART_INTER):
    {
        cart_client_inter_->cancelAllGoals();
        break;
    }
    case(ActionClient::CART_PLAN):
    {
        cart_client_plan_->cancelAllGoals();
        break;
    }
    case(ActionClient::CART_PATH):
    {
        cart_traj_client_->cancelAllGoals();
        break;
    }
    case(ActionClient::CYL_DIRECT):
    {
        cyl_client_direct_->cancelAllGoals();
        break;
    }
    case(ActionClient::CYL_INTER):
    {
        cyl_client_inter_->cancelAllGoals();
        break;
    }
    case(ActionClient::CYL_PLAN):
    {
        cyl_client_plan_->cancelAllGoals();
        break;
    }
    case(ActionClient::CYL_PATH):
    {
        cyl_traj_client_->cancelAllGoals();
        break;
    }
    case(ActionClient::JOINT_DIRECT):
    {
        jnt_client_direct_->cancelAllGoals();
        break;
    }
    case(ActionClient::JOINT_INTER):
    {
        jnt_client_inter_->cancelAllGoals();
        break;
    }
    case(ActionClient::JOINT_PLAN):
    {
        jnt_client_plan_->cancelAllGoals();
        break;
    }
    case(ActionClient::JOINT_PATH):
    {
        jnt_traj_client_->cancelAllGoals();
        break;
    }
    case(ActionClient::NAME_DIRECT):
    {
        name_client_direct_->cancelAllGoals();
        break;
    }
    case(ActionClient::NAME_INTER):
    {
        name_client_inter_->cancelAllGoals();
        break;
    }
    case(ActionClient::NAME_PLAN):
    {
        name_client_plan_->cancelAllGoals();
        break;
    }
    case(ActionClient::NAME_PATH):
    {
        name_traj_client_->cancelAllGoals();
        break;
    }
    }
}

//########## ACTION SUCCEEDED ##########################################################################################
bool YoubotArm::actionSucceeded()
{
    switch(active_client_)
    {
    case(ActionClient::CART_DIRECT):
    {
        return cart_client_direct_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::CART_INTER):
    {
        return cart_client_inter_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::CART_PLAN):
    {
        return cart_client_plan_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::CART_PATH):
    {
        return cart_traj_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::CYL_DIRECT):
    {
        return cyl_client_direct_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::CYL_INTER):
    {
        return cyl_client_inter_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::CYL_PLAN):
    {
        return cyl_client_plan_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::CYL_PATH):
    {
        return cyl_traj_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::JOINT_DIRECT):
    {
        return jnt_client_direct_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::JOINT_INTER):
    {
        return jnt_client_inter_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::JOINT_PLAN):
    {
        return jnt_client_plan_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::JOINT_PATH):
    {
        return jnt_traj_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::NAME_DIRECT):
    {
        return name_client_direct_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::NAME_INTER):
    {
        return name_client_inter_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::NAME_PLAN):
    {
        return name_client_plan_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(ActionClient::NAME_PATH):
    {
        return name_traj_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    }
}

//########## ENABLE CLIENT #############################################################################################
void YoubotArm::enableClient(int client)
{
    is_enabled_[client] = true;
}

//########## DISABLE CLIENT ############################################################################################
void YoubotArm::disableClient(int client)
{
    is_enabled_[client] = false;
}

//########## ENABLE ALL CLIENTS ########################################################################################
void YoubotArm::enableAllClients()
{
    for(std::map<int,bool>::iterator it = is_enabled_.begin(); it != is_enabled_.end(); ++it)
    {
        it->second = true;
    }
}

//########## DISABLE ALL CLIENTS #######################################################################################
void YoubotArm::disableAllClients()
{
    for(std::map<int,bool>::iterator it = is_enabled_.begin(); it != is_enabled_.end(); ++it)
    {
        it->second = false;
    }
}

//########## SET MAX VELOCITY ##########################################################################################
bool YoubotArm::setMaxVelocity(luh_youbot_kinematics::CartesianVelocity velocity)
{
    luh_youbot_msgs::SetCartesianVelocity srv;
    srv.request.max_velocity.x = velocity.x();
    srv.request.max_velocity.y = velocity.y();
    srv.request.max_velocity.z = velocity.z();
    srv.request.max_velocity.theta = velocity.theta();
    srv.request.max_velocity.q5 = velocity.q5();

    return set_cart_vel_client_.call(srv);
}

//########## SET MAX VELOCITY ##########################################################################################
bool YoubotArm::setMaxVelocity(luh_youbot_kinematics::CylindricVelocity velocity)
{
    luh_youbot_msgs::SetCylindricVelocity srv;
    srv.request.max_velocity.q1 = velocity.q1();
    srv.request.max_velocity.r = velocity.r();
    srv.request.max_velocity.z = velocity.z();
    srv.request.max_velocity.theta = velocity.theta();
    srv.request.max_velocity.q5 = velocity.q5();

    return set_cyl_vel_client_.call(srv);
}

//########## SET MAX VELOCITY ##########################################################################################
bool YoubotArm::setMaxVelocity(luh_youbot_kinematics::JointVelocity velocity)
{
    luh_youbot_msgs::SetJointVelocity srv;
    srv.request.max_velocity.q1 = velocity.q1();
    srv.request.max_velocity.q2 = velocity.q2();
    srv.request.max_velocity.q3 = velocity.q3();
    srv.request.max_velocity.q4 = velocity.q4();
    srv.request.max_velocity.q5 = velocity.q5();

    return set_jnt_vel_client_.call(srv);
}

}
