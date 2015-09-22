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
 * @brief  Source file for the YoubotGripper class.
 */

#include "luh_youbot_controller_api/youbot_gripper.h"

namespace youbot_api
{

//########## CONSTRUCTOR ###############################################################################################
YoubotGripper::YoubotGripper():
    is_busy_(false),
    grip_object_client_(NULL),
    set_gripper_client_(NULL)
{

}

//########## DESTRUCTOR ################################################################################################
YoubotGripper::~YoubotGripper()
{
    delete grip_object_client_;
    delete set_gripper_client_;
}

//########## INIT ######################################################################################################
void YoubotGripper::init(ros::NodeHandle &node)
{
    // save pointer to node handle
    node_ = &node;

    ROS_INFO("Waiting for gripper clients...");

    // create action clients
    grip_object_client_ = new GripObjectClient("arm_1/grip_object", true);
    grip_object_client_->waitForServer();

    set_gripper_client_ = new SetGripperClient("arm_1/set_gripper", true);
    set_gripper_client_->waitForServer();

    ROS_INFO("Gripper action clients initialised.");
}

//########## SET GRIPPER ###############################################################################################
void YoubotGripper::setWidth(double value, bool is_relative)
{
    luh_youbot_msgs::SetGripperGoal goal;
    goal.gripper_width = value;
    goal.is_relative = is_relative;

    active_client_ = SET_GRIPPER;
    is_busy_ = true;
    set_gripper_client_->sendGoal(goal, boost::bind(&YoubotGripper::onSetGripperActionDone, this, _1, _2));
}

//########## GRIP OBJECT ###############################################################################################
void YoubotGripper::gripObject(std::string object_name)
{
    luh_youbot_msgs::GripObjectGoal goal;
    goal.object_name = object_name;

    active_client_ = GRIP_OBJECT;
    is_busy_ = true;
    grip_object_client_->sendGoal(goal, boost::bind(&YoubotGripper::onGripObjectActionDone, this, _1, _2));
}

//########## OPEN ######################################################################################################
void YoubotGripper::open()
{
    luh_youbot_msgs::GripObjectGoal goal;
    goal.object_name = "OPEN";

    active_client_ = GRIP_OBJECT;
    is_busy_ = true;
    grip_object_client_->sendGoal(goal, boost::bind(&YoubotGripper::onGripObjectActionDone, this, _1, _2));
}

//########## WAIT FOR CURRENT ACTION ###################################################################################
bool YoubotGripper::waitForCurrentAction(double timeout)
{
    if(!is_busy_)
        return true;

    bool done = false;
    switch(active_client_)
    {
    case(GRIP_OBJECT):
    {
        done = grip_object_client_->waitForResult(ros::Duration(timeout));
        break;
    }
    case(SET_GRIPPER):
    {
        done = set_gripper_client_->waitForResult(ros::Duration(timeout));
        break;
    }
    }

    if(done)
    {
        is_busy_ = false;
    }

    return done;
}

//########## ON SET GRIPPER ACTION DONE ################################################################################
void YoubotGripper::onSetGripperActionDone(const actionlib::SimpleClientGoalState& state,
                                           const luh_youbot_msgs::SetGripperResultConstPtr& result)
{
   is_busy_ = false;
}

//########## ON GRIP OBJECT ACTION DONE ################################################################################
void YoubotGripper::onGripObjectActionDone(const actionlib::SimpleClientGoalState& state,
                                           const luh_youbot_msgs::GripObjectResultConstPtr& result)
{
   is_busy_ = false;
}

//########## ABORT CURRENT ACTION ######################################################################################
void YoubotGripper::abortCurrentAction()
{
    switch(active_client_)
    {
    case(SET_GRIPPER):
    {
        set_gripper_client_->cancelAllGoals();
        break;
    }
    case(GRIP_OBJECT):
    {
        grip_object_client_->cancelAllGoals();
        break;
    }
    }
}

//########## ACTION SUCCEEDED ##########################################################################################
bool YoubotGripper::actionSucceeded()
{
    switch(active_client_)
    {
    case(SET_GRIPPER):
    {
        return set_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    case(GRIP_OBJECT):
    {
        return grip_object_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    }
}

}
