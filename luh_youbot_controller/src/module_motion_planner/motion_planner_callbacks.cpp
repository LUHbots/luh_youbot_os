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

#include "luh_youbot_controller/module_motion_planner/motion_planner.h"

using namespace luh_youbot_kinematics;

//############### CALLBACK: PREDEFINED POSE ############################################################################
bool ModuleMotionPlanner::predefPoseCallback(luh_youbot_msgs::ToPredefinedPose::Request &req,
                                             luh_youbot_msgs::ToPredefinedPose::Response &res)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!isReady())
        return false;

    if(pose_map_.find(req.pose_name) == pose_map_.end()
            && predefined_poses_.find(req.pose_name) == predefined_poses_.end())
    {
        ROS_ERROR("Requested pose '%s' does not exist.", req.pose_name.c_str());
        return false;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endPredefinedPoseService;

    // === GET END POINT ===
    if(pose_map_.find(req.pose_name) == pose_map_.end())
    {
        end_pose_ = predefined_poses_[req.pose_name];
        end_pose_.subtractOffset();
    }
    else
        end_pose_ = pose_map_[req.pose_name]->joint_position;

    // === PLAN PATH ===
    JointPosition current_position = youbot_->arm()->getJointPosition();
    current_position.subtractOffset();
    Path path = getPath(current_position, end_pose_);

    // === START MOVEMENT ===
    startMovement(path);

    return true;
}

//############### CALLBACK: LIST POSES #################################################################################
bool ModuleMotionPlanner::listPosesCallback(luh_youbot_msgs::ListPoses::Request &req,
                                            luh_youbot_msgs::ListPoses::Response &res)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    std::map<std::string, bool> listed;
    std::stringstream ss;

    for (std::map<std::string, Pose*>::iterator it=pose_map_.begin(); it!=pose_map_.end(); ++it)
    {
        std::string name = it->first;
        if(listed.find(name) == listed.end())
        {
            listed[name] = true;
            ss << std::endl << name;
        }
    }
    for (youbot_poses::PoseMap::iterator it=predefined_poses_.begin(); it!=predefined_poses_.end(); ++it)
    {
        std::string name = it->first;
        if(listed.find(name) == listed.end())
        {
            listed[name] = true;
            ss << std::endl << name;
        }
    }

    res.poses = ss.str();

    return true;
}

//############### CALLBACK: JOINT POSE #################################################################################
void ModuleMotionPlanner::jointPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveToJointPoseGoal> goal = joint_pose_server_->acceptNewGoal();

    if(!isReady())
    {
        joint_pose_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endJointPoseAction;

    // === GET END POINT ===
    end_pose_.setQ1(goal->pose.q1);
    end_pose_.setQ2(goal->pose.q2);
    end_pose_.setQ3(goal->pose.q3);
    end_pose_.setQ4(goal->pose.q4);
    end_pose_.setQ5(goal->pose.q5);

    if(goal->pose_is_relative)
        end_pose_ += youbot_->arm()->getJointPosition();

    // === SAFETY CHECK ===
    if(!end_pose_.isReachable())
    {
        ROS_ERROR("Position is unreachable.");
        joint_pose_server_->setAborted();
        return;
    }

    // === PLAN PATH ===
    JointPosition current_position = youbot_->arm()->getJointPosition();
    current_position.subtractOffset();
    end_pose_.subtractOffset();
    Path path = getPath(current_position, end_pose_);

    // === START MOVEMENT ===
    startMovement(path);
}

//############### CALLBACK: CARTESIAN POSE #############################################################################
void ModuleMotionPlanner::cartesianPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveToCartesianPoseGoal> goal = cartesian_pose_server_->acceptNewGoal();

    if(!isReady())
    {
        cartesian_pose_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endCartesianPoseAction;

    // === GET CARTESIAN END POINT ===
    CartesianPosition end;
    try
    {
        end = CartesianPosition::fromMsg(goal->pose, goal->pose_is_relative,
                                         youbot_->arm()->getJointPosition(), tf_listener_);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        cartesian_pose_server_->setAborted();
        return;
    }

    // === GET JOINTSPACE END POINT ===
    end_pose_ = end.toJointspace();

    // === SAFETY CHECK ===
    if(!(end_pose_.isReachable() && end.isReachable()))
    {
        ROS_ERROR("Position is unreachable.");
        cartesian_pose_server_->setAborted();
        return;
    }

    // === PLAN PATH ===
    JointPosition current_position = youbot_->arm()->getJointPosition();
    current_position.subtractOffset();
    end_pose_.subtractOffset();
    Path path = getPath(current_position, end_pose_);

    // === START MOVEMENT ===
    startMovement(path);
}

//############### CALLBACK: CYLINDRIC POSE #############################################################################
void ModuleMotionPlanner::cylindricPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveToCylindricPoseGoal> goal = cylindric_pose_server_->acceptNewGoal();

    ROS_INFO("MMP: Got request to move to cylindrical pose [%f;%f;%f;%f;%f]",
             goal->pose.q1,
             goal->pose.r,
             goal->pose.z,
             goal->pose.theta,
             goal->pose.q5);

    if(!isReady())
    {
        cylindric_pose_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endCylindricPoseAction;

    // === GET END CYLINDRIC POINT ===
    CylindricPosition position_command;
    position_command.setQ1(goal->pose.q1);
    position_command.setR(goal->pose.r);
    position_command.setZ(goal->pose.z);
    position_command.setTheta(goal->pose.theta);
    position_command.setQ5(goal->pose.q5);

    if(goal->pose_is_relative)
    {
        position_command += youbot_->arm()->getJointPosition().toCylindric();
    }

    // === GET JOINTSPACE END POINT ===
    end_pose_ = position_command.toJointspace();

    // === SAFETY CHECK ===
    if(!end_pose_.isReachable())
    {
        ROS_ERROR("Position exceeds joint limitations.");
        cylindric_pose_server_->setAborted();
        return;
    }
    if(!position_command.isReachable())
    {
        ROS_ERROR("Position is unreachable.");
        cylindric_pose_server_->setAborted();
        return;
    }

    // === PLAN PATH ===
    JointPosition current_position = youbot_->arm()->getJointPosition();
    current_position.subtractOffset();
    end_pose_.subtractOffset();
    Path path = getPath(current_position, end_pose_);

    // === START MOVEMENT ===
    startMovement(path);
}

//############### CALLBACK: NAMED POSE #################################################################################
void ModuleMotionPlanner::namedPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveToNamedPoseGoal> goal = named_pose_server_->acceptNewGoal();

    if(!isReady())
    {
        named_pose_server_->setAborted();
        return;
    }

    if(pose_map_.find(goal->pose_name) == pose_map_.end()
            && predefined_poses_.find(goal->pose_name) == predefined_poses_.end())
    {
        ROS_ERROR("Requested pose '%s' does not exist.", goal->pose_name.c_str());
        named_pose_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endNamedPoseAction;

    // === GET END POINT ===
    if(pose_map_.find(goal->pose_name) == pose_map_.end())
    {
        end_pose_ = predefined_poses_[goal->pose_name];
        end_pose_.subtractOffset();
    }
    else
    {
        end_pose_ = pose_map_[goal->pose_name]->joint_position;
    }


    // === PLAN PATH ===
    JointPosition current_position = youbot_->arm()->getJointPosition();
    current_position.subtractOffset();
    Path path = getPath(current_position, end_pose_);

    // === START MOVEMENT ===
    startMovement(path);
}

//############### CALLBACK: JOINT PATH ###########################################################################
void ModuleMotionPlanner::jointPathCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveJointPathGoal> goal = joint_path_server_->acceptNewGoal();

    if(!isReady())
    {
        joint_path_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endJointPathAction;

    // === GET PATH ===
    Path path;
    path.resize(goal->path.poses.size()+1);

    for(uint i=0; i<path.size()-1; i++)
    {
        path[i+1].setQ1(goal->path.poses[i].q1);
        path[i+1].setQ2(goal->path.poses[i].q2);
        path[i+1].setQ3(goal->path.poses[i].q3);
        path[i+1].setQ4(goal->path.poses[i].q4);
        path[i+1].setQ5(goal->path.poses[i].q5);

        // === SAFETY CHECK ===
        if(!path[i+1].isReachable())
        {
            ROS_ERROR("Position is unreachable.");
            joint_path_server_->setAborted();
            return;
        }

        path[i+1].subtractOffset();
    }

    end_pose_ = path.back();

    // === START MOVEMENT ===
    startMovement(path);
}

//############### CALLBACK: CARTESIAN PATH #######################################################################
void ModuleMotionPlanner::cartesianPathCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveCartesianPathGoal> goal
            = cartesian_path_server_->acceptNewGoal();

    if(!isReady())
    {
        cartesian_path_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endCartesianPathAction;

    // === GET PATH ===
    Path path;
    path.resize(goal->path.poses.size()+1);

    try
    {
        for(uint i=0; i<path.size()-1; i++)
        {
            CartesianPosition pos = CartesianPosition::fromMsg(goal->path.poses[i], false,
                                                               youbot_->arm()->getJointPosition(), tf_listener_);

            // === GET JOINTSPACE POSE ===
            path[i+1] = pos.toJointspace();

            // === SAFETY CHECK ===
            if(!(path[i+1].isReachable() && pos.isReachable()))
            {
                ROS_ERROR("Position is unreachable.");
                cartesian_path_server_->setAborted();
                return;
            }

            path[i+1].subtractOffset();
        }
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        cartesian_path_server_->setAborted();
        return;
    }
    end_pose_ = path.back();

    // === START MOVEMENT ===
    startMovement(path);
}

//############### CALLBACK: CYLINDRIC PATH #######################################################################
void ModuleMotionPlanner::cylindricPathCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveCylindricPathGoal> goal
            = cylindric_path_server_->acceptNewGoal();

    if(!isReady())
    {
        cylindric_path_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endCylindricPathAction;

    // === GET PATH ===
    Path path;
    path.resize(goal->path.poses.size()+1);

    for(uint i=0; i<path.size()-1; i++)
    {
        CylindricPosition pos;
        pos.setQ1(goal->path.poses[i].q1);
        pos.setR(goal->path.poses[i].r);
        pos.setZ(goal->path.poses[i].z);
        pos.setTheta(goal->path.poses[i].theta);
        pos.setQ5(goal->path.poses[i].q5);

        // === GET JOINTSPACE POSE ===
        path[i+1] = pos.toJointspace();

        // === SAFETY CHECK ===
        if(!(path[i+1].isReachable() && pos.isReachable()))
        {
            ROS_ERROR("Position is unreachable.");
            cylindric_path_server_->setAborted();
            return;
        }

        path[i+1].subtractOffset();
    }
    end_pose_ = path.back();

    // === START MOVEMENT ===
    startMovement(path);
}

//############### CALLBACK: NAMED PATH ###########################################################################
void ModuleMotionPlanner::namedPathCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    // === GET GOAL ===
    boost::shared_ptr<const luh_youbot_msgs::MoveNamedPathGoal> goal = named_path_server_->acceptNewGoal();

    if(!isReady())
    {
        named_path_server_->setAborted();
        return;
    }

    // === SET END CALLBACK ===
    endAction = &ModuleMotionPlanner::endNamedPathAction;

    // === GET PATH ===
    Path path;
    path.resize(goal->path.poses.size()+1);

    for(uint i=0; i<path.size()-1; i++)
    {
        if(pose_map_.find(goal->path.poses[i]) == pose_map_.end()
                && predefined_poses_.find(goal->path.poses[i]) == predefined_poses_.end())
        {
            ROS_WARN("Unknown pose: '%s'", goal->path.poses[i].c_str());
            named_path_server_->setAborted();
            return;
        }

        if(pose_map_.find(goal->path.poses[i]) == pose_map_.end())
        {
            path[i+1] = predefined_poses_[goal->path.poses[i]];
            path[i+1].subtractOffset();
        }
        else
            path[i+1] = pose_map_[goal->path.poses[i]]->joint_position;
    }

    end_pose_ = path.back();

    // === START MOVEMENT ===
    startMovement(path);
}

//############### END JOINT POSE ACTION ################################################################################
void ModuleMotionPlanner::endJointPoseAction(bool success)
{
    if(success)
        joint_pose_server_->setSucceeded();
    else
        joint_pose_server_->setAborted();

}

//############### END CARTESIAN POSE ACTION ############################################################################
void ModuleMotionPlanner::endCartesianPoseAction(bool success)
{
    if(success)
        cartesian_pose_server_->setSucceeded();
    else
        cartesian_pose_server_->setAborted();
}
//############### END JOINT POSE ACTION ################################################################################
void ModuleMotionPlanner::endCylindricPoseAction(bool success)
{
    if(success)
        cylindric_pose_server_->setSucceeded();
    else
        cylindric_pose_server_->setAborted();
}
//############### END NAMED POSE ACTION ################################################################################
void ModuleMotionPlanner::endNamedPoseAction(bool success)
{
    if(success)
        named_pose_server_->setSucceeded();
    else
        named_pose_server_->setAborted();
}

//############### END JOINT PATH ACTION ##########################################################################
void ModuleMotionPlanner::endJointPathAction(bool success)
{
    if(success)
        joint_path_server_->setSucceeded();
    else
        joint_path_server_->setAborted();
}

//############### END CARTESIAN PATH ACTION ######################################################################
void ModuleMotionPlanner::endCartesianPathAction(bool success)
{
    if(success)
        cartesian_path_server_->setSucceeded();
    else
        cartesian_path_server_->setAborted();
}

//############### END CYLINDRIC PATH ACTION ######################################################################
void ModuleMotionPlanner::endCylindricPathAction(bool success)
{
    if(success)
        cylindric_path_server_->setSucceeded();
    else
        cylindric_path_server_->setAborted();
}

//############### END NAMED PATH ACTION ##########################################################################
void ModuleMotionPlanner::endNamedPathAction(bool success)
{
    if(success)
        named_path_server_->setSucceeded();
    else
        named_path_server_->setAborted();
}

//############### END PREDEFINED POSE SERVICE ##########################################################################
void ModuleMotionPlanner::endPredefinedPoseService(bool success)
{
    //empty
}

//############### PREEMPT CALLBACK #####################################################################################
void ModuleMotionPlanner::preemptCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ROS_WARN("MMP: Movement aborted.");

    if(joint_path_server_->isActive())
        joint_path_server_->setPreempted();
    if(cartesian_path_server_->isActive())
        cartesian_path_server_->setPreempted();
    if(cylindric_path_server_->isActive())
        cylindric_path_server_->setPreempted();
    if(named_path_server_->isActive())
        named_path_server_->setPreempted();
    if(joint_pose_server_->isActive())
        joint_pose_server_->setPreempted();
    if(cartesian_pose_server_->isActive())
        cartesian_pose_server_->setPreempted();
    if(cylindric_pose_server_->isActive())
        cylindric_pose_server_->setPreempted();
    if(named_pose_server_->isActive())
        named_pose_server_->setPreempted();

    controller_->stop();

    // === SET CURRENT POSE ===
    youbot_->arm()->setJointPositions(youbot_->arm()->getJointPosition());

    status_ = STATUS_IDLE;
    arm_is_busy_ = false;

}
