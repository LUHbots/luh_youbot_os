/* *****************************************************************
 *
 * luh_youbot_manipulation_api
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
 * @author Simon Aden (simon.aden@mailbox.org)
 * @date   November 2014
 *
 * @brief  Source file for the YoubotBase class.
 */

#include "luh_youbot_manipulation_api/youbot_base.h"
#include <luh_youbot_msgs/GetBasePose.h>

namespace manipulation_api
{

//########## CONSTRUCTOR ###############################################################################################
YoubotBase::YoubotBase():
    move_base_client_(NULL),
    is_busy_(false)
{

}

//########## DESTRUCTOR ################################################################################################
YoubotBase::~YoubotBase()
{
    delete move_base_client_;
}

//########## INIT ######################################################################################################
void YoubotBase::init(ros::NodeHandle &node)
{
    // save pointer to node handle
    node_ = &node;

    ROS_INFO("Waiting for base action servers...");

    // create action client
    move_base_client_  = new MoveBaseClient("youbot_base/move", true);
    move_base_client_->waitForServer();

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

    bool done;

    done = move_base_client_->waitForResult(ros::Duration(timeout));

    if(done)
        is_busy_ = false;

    return done;
}

//########## ON JOINT ACTION DONE ######################################################################################
void YoubotBase::onMoveBaseActionDone(const actionlib::SimpleClientGoalState& state,
                                      const luh_youbot_msgs::MoveBaseResultConstPtr& result)
{
    is_busy_ = false;

    traveled_distance_.x += result->x;
    traveled_distance_.y += result->y;
    traveled_distance_.theta += result->theta;
}

//########## ABORT CURRENT ACTION ######################################################################################
void YoubotBase::abortCurrentAction()
{
    move_base_client_->cancelAllGoals();
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
    return move_base_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;

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
} // namespace manipulation_api
