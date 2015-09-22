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
 * @brief  Source file for the YoubotBase class.
 */

#include "luh_youbot_controller_api/youbot_base.h"
#include <luh_youbot_msgs/GetBasePose.h>

namespace youbot_api
{

//########## CONSTRUCTOR ###############################################################################################
YoubotBase::YoubotBase():
    move_base_client_(NULL),
    approach_client_(NULL),
    is_busy_(false)
{

}

//########## DESTRUCTOR ################################################################################################
YoubotBase::~YoubotBase()
{
    delete move_base_client_;
    delete approach_client_;
}

//########## INIT ######################################################################################################
void YoubotBase::init(ros::NodeHandle &node)
{
    // save pointer to node handle
    node_ = &node;

    ROS_INFO("Waiting for base action servers...");

    // create action clients
    move_base_client_  = new MoveBaseClient("youbot_base/move", true);
    move_base_client_->waitForServer();
    approach_client_ = new ApproachClient("youbot_base/approach", true);
    approach_client_->waitForServer();

    // service client
    get_pose_client_ = node.serviceClient<luh_youbot_msgs::GetBasePose>("youbot_base/get_pose");

    // initialise traveled distance
    traveled_distance_.x = 0;
    traveled_distance_.y = 0;
    traveled_distance_.theta = 0;

    ROS_INFO("Base action clients initialised.");
}

//########## MOVE BASE #################################################################################################
void YoubotBase::move(double x, double y, double theta)
{
    luh_youbot_msgs::MoveBaseGoal goal;
    goal.x = x;
    goal.y = y;
    goal.theta = theta;

    active_client_ = MOVE_BASE;
    is_busy_ = true;
    move_base_client_->sendGoal(goal, boost::bind(&YoubotBase::onMoveBaseActionDone, this, _1, _2));

}

//########## APPROACH ##################################################################################################
void YoubotBase::approach(double front, double right)
{
    luh_youbot_msgs::ApproachBaseGoal goal;
    if(right < 0)
    {
        goal.left = -right;
        goal.right = 0;
    }
    else
    {
        goal.left = 0;
        goal.right = right;
    }
    if(front < 0)
    {
        goal.back = -front;
        goal.front = 0;
    }
    else
    {
        goal.back = 0;
        goal.front = front;
    }

    active_client_ = APPROACH;
    is_busy_ = true;
    approach_client_->sendGoal(goal, boost::bind(&YoubotBase::onApproachActionDone, this, _1, _2));

}

//########## MOVE BASE #################################################################################################
void YoubotBase::move(Pose2D pose)
{
    move(pose.x, pose.y, pose.theta);
}

//########## WAIT FOR ACTION ###########################################################################################
bool YoubotBase::waitForCurrentAction(double timeout)
{
    if(!is_busy_)
        return true;

    bool done = false;

    if(active_client_ == MOVE_BASE)
        done = move_base_client_->waitForResult(ros::Duration(timeout));
    if(active_client_ == APPROACH)
        done = approach_client_->waitForResult(ros::Duration(timeout));

    if(done)
        is_busy_ = false;

    return done;
}

//########## ON MOVE BASE ACTION DONE ##################################################################################
void YoubotBase::onMoveBaseActionDone(const actionlib::SimpleClientGoalState& state,
                                      const luh_youbot_msgs::MoveBaseResultConstPtr& result)
{
    is_busy_ = false;

    traveled_distance_.x += result->x;
    traveled_distance_.y += result->y;
    traveled_distance_.theta += result->theta;
}

//########## ON APPROACH ACTION DONE ###################################################################################
void YoubotBase::onApproachActionDone(const actionlib::SimpleClientGoalState& state,
                                      const luh_youbot_msgs::ApproachBaseResultConstPtr& result)
{
    is_busy_ = false;

    traveled_distance_.x += result->moved_distance.x;
    traveled_distance_.y += result->moved_distance.y;
    traveled_distance_.theta += result->moved_distance.theta;
}

//########## ABORT CURRENT ACTION ######################################################################################
void YoubotBase::abortCurrentAction()
{
    if(active_client_ == MOVE_BASE)
        move_base_client_->cancelAllGoals();
    if(active_client_ == APPROACH)
        approach_client_->cancelAllGoals();
}

//########## RESET TRAVELED DISTANCE ###################################################################################
void YoubotBase::resetTraveledDistance()
{
    traveled_distance_.x = 0;
    traveled_distance_.y = 0;
    traveled_distance_.theta = 0;
}

//########## GET TRAVELED DISTANCE #####################################################################################
Pose2D YoubotBase::getTraveledDistance()
{
    return traveled_distance_;
}

//########## ACTION SUCCEEDED ##########################################################################################
bool YoubotBase::actionSucceeded()
{
    if(active_client_ == MOVE_BASE)
        return move_base_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;

    if(active_client_ == APPROACH)
        return approach_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

//########## GET CURRENT POSE ##########################################################################################
Pose2D YoubotBase::getCurrentPose()
{
    luh_youbot_msgs::GetBasePose srv;
    get_pose_client_.call(srv);

    Pose2D ans;
    ans.x = srv.response.x;
    ans.y = srv.response.y;
    ans.theta = srv.response.theta;

    return ans;
}
} // namespace youbot_api
