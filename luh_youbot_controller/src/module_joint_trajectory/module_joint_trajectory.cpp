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

#include "luh_youbot_controller/module_joint_trajectory/module_joint_trajectory.h"

namespace ykin = luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
ModuleJointTrajectory::ModuleJointTrajectory(): ControllerModule(),
    server_(NULL),
    has_active_goal_(false)
{
    feedback_.actual.positions.assign(ykin::N_JOINTS, 0.0);
    feedback_.actual.velocities.assign(ykin::N_JOINTS, 0.0);
    feedback_.actual.accelerations.assign(ykin::N_JOINTS, 0.0);
    feedback_.actual.effort.assign(ykin::N_JOINTS, 0.0);
    feedback_.error.positions.assign(ykin::N_JOINTS, 0.0);
    feedback_.error.velocities.assign(ykin::N_JOINTS, 0.0);
    feedback_.error.accelerations.assign(ykin::N_JOINTS, 0.0);
    feedback_.error.effort.assign(ykin::N_JOINTS, 0.0);
    feedback_.desired.positions.assign(ykin::N_JOINTS, 0.0);
    feedback_.desired.velocities.assign(ykin::N_JOINTS, 0.0);
    feedback_.desired.accelerations.assign(ykin::N_JOINTS, 0.0);
    feedback_.desired.effort.assign(ykin::N_JOINTS, 0.0);
}

//########## DESTRUCTOR ################################################################################################
ModuleJointTrajectory::~ModuleJointTrajectory()
{
    delete server_;
}

//########## INITIALIZATION ############################################################################################
void ModuleJointTrajectory::init()
{
    ROS_INFO("MJT: Initialising Joint Trajectory Module...");
    activated_ = true;

    // initialise ROS communication
    server_ = new TrajectoryActionServer(*node_, "arm_1/follow_joint_trajectory", false);
    server_->registerGoalCallback(boost::bind(&ModuleJointTrajectory::goalCallback, this));
    server_->start();

    // === PARAMETERS ===
    default_path_tolerance_.assign(ykin::N_JOINTS, 5.0);
    ros::param::get("module_joint_trajectory/path_tolerance", default_path_tolerance_);
    default_path_tolerance_ *= M_PI/180.0;
    default_goal_tolerance_.assign(ykin::N_JOINTS, 0.1);
    ros::param::get("module_joint_trajectory/goal_tolerance", default_goal_tolerance_);
    default_goal_tolerance_ *= M_PI / 180.0;
    node_->param("module_joint_trajectory/goal_time_tolerance", goal_time_tolerance_, 1.0);

    ROS_INFO("MJT: Joint Trajectory Module initialised.");
}

//########## UPDATE ####################################################################################################
void ModuleJointTrajectory::update()
{
    if(!activated_ || !has_active_goal_)
        return;

    // === CHECK PREEMPTION ===
    if(server_->isPreemptRequested())
    {
        ROS_WARN("MJT: Action has been preempted.");
        result_.error_code = result_.SUCCESSFUL;
        result_.error_string = "Action has been preempted.";
        server_->setPreempted(result_);
        has_active_goal_ = false;
        return;
    }

    // === GET CURRENT TIME AND POSITION ===
    ros::Duration passed_time = ros::Time::now() - start_time_;
    actual_position_ = youbot_->arm()->getJointPosition();

    // === FIND NEXT TRAJECTORY POINT ===
    while(next_point_->time < passed_time && !trajectory_ended_)
    {
        last_point_++;
        next_point_++;

        if(next_point_ == trajectory_.end())
            trajectory_ended_ = true;
    }

    // === INTERPOLATE TRAJECTORY POINTS ===
    if(!trajectory_ended_)
    {
        // interpolate
        double r = (passed_time - last_point_->time).toSec() / (next_point_->time - last_point_->time).toSec();
        desired_position_ = last_point_->position * (1-r) + next_point_->position * r;
        position_error_ = actual_position_ - desired_position_;

        // check position error
        for(uint i=0; i<ykin::N_JOINTS; i++)
        {
            if(fabs(position_error_[i]) > path_tolerance_[i])
            {
                ROS_INFO("MJT: Position error: %f", position_error_[i]*180/M_PI);
                result_.error_code = result_.PATH_TOLERANCE_VIOLATED;
                std::stringstream ss;
                ss << "Path tolerance in " << feedback_.joint_names[i] << " violated.";
                result_.error_string = ss.str();

                server_->setAborted(result_);
                has_active_goal_ = false;
                ROS_ERROR("MJT: %s", result_.error_string.c_str());
                return;
            }
        }
    }

    // === WAIT TO REACH GOAL ===
    else
    {
        desired_position_ = trajectory_.back().position;
        position_error_ = actual_position_ - desired_position_;

        bool goal_reached = true;

        // check goal tolerance
        for(uint i=0; i<ykin::N_JOINTS; i++)
        {
            if(fabs(position_error_[i]) > goal_tolerance_[i])
            {
                goal_reached = false;
                break;
            }
        }

        if(goal_reached)
        {
            // success
            result_.error_code = result_.SUCCESSFUL;
            result_.error_string = "Action finished successfully.";
            server_->setSucceeded(result_);
            has_active_goal_ = false;
            ROS_INFO("MJT: Goal position reached.");
            return;
        }

        if((passed_time - trajectory_.back().time).toSec() > goal_time_tolerance_)
        {
            // abort
            result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
            std::stringstream ss;
            ss << "Reaching the goal tolerance took more than " << goal_time_tolerance_ << " seconds.";
            result_.error_string = ss.str();
            server_->setAborted(result_);
            has_active_goal_ = false;
            ROS_ERROR("MJT: %s", result_.error_string.c_str());
            return;
        }
    }

    // === SEND FEEDBACK ===
    ykin::JointPosition desired = desired_position_;
    ykin::JointPosition actual = actual_position_;
    desired.subtractOffset();
    actual.subtractOffset();
    feedback_.actual.time_from_start = passed_time;
    feedback_.desired.time_from_start = passed_time;
    feedback_.error.time_from_start = passed_time;
    for(uint i=0; i<ykin::N_JOINTS; i++)
    {
        feedback_.actual.positions[i] = actual[i];
        feedback_.desired.positions[i] = desired[i];
        feedback_.error.positions[i] = position_error_[i];
    }

    youbot_->arm()->setJointPositions(desired_position_);
}

//########## ACTIVATE ##################################################################################################
void ModuleJointTrajectory::activate()
{
    activated_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleJointTrajectory::deactivate()
{
    activated_ = false;
}

//########## EMERGENCY STOP ############################################################################################
void ModuleJointTrajectory::emergencyStop()
{
    if(server_->isActive())
    {
        result_.error_code = result_.PATH_TOLERANCE_VIOLATED;
        result_.error_string = "Emergency Stop has been requested.";
        server_->setAborted(result_);
    }
    has_active_goal_ = false;
}

//########## EXECUTE CALLBACK ##########################################################################################
void ModuleJointTrajectory::goalCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ROS_INFO("==== Module Joint Trajectory ====");
    boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> goal = server_->acceptNewGoal();

    // === INITIALISATION ===
    trajectory_.clear();

    if(goal->trajectory.header.stamp == ros::Time(0))
        start_time_ = ros::Time::now();
    else
        start_time_ = goal->trajectory.header.stamp;

    // === SAVE TRAJECTORY POSITIONS ===
    TrajectoryPoint start_point;
    start_point.position = youbot_->arm()->getJointPosition();
    start_point.time = ros::Duration(0);

    for(uint i=0; i<goal->trajectory.points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint goal_point = goal->trajectory.points[i];
        TrajectoryPoint traj_point;
        traj_point.time = goal_point.time_from_start;

        for(uint j=0; j<goal_point.positions.size(); j++)
        {
            traj_point.position[j] = goal_point.positions[j];
        }

        traj_point.position.addOffset();

        if(traj_point.position.isReachable())
            trajectory_.push_back(traj_point);
        else
            ROS_WARN("MJT: Trajectory Point %d is unreachable and will be ignored: [%f;%f;%f;%f;%f]",
                     i,
                     traj_point.position.q1(),
                     traj_point.position.q2(),
                     traj_point.position.q3(),
                     traj_point.position.q4(),
                     traj_point.position.q5());
    }

    // === ERROR CHECK ===
    if(trajectory_.size() < 2)
    {
        ROS_ERROR("MJT: Trajectory is empty.");
        result_.error_code = result_.INVALID_GOAL;
        if(goal->trajectory.points.empty())
            result_.error_string = "Trajectory is empty.";
        else
            result_.error_string = "No valid trajectory points given.";
        server_->setAborted(result_);
        return;
    }

    // === GET PATH TOLERANCE ===
    if(goal->path_tolerance.size() != ykin::N_JOINTS)
        path_tolerance_ = default_path_tolerance_;
    else
    {
        for(uint i=0; i<ykin::N_JOINTS; i++)
        {
            double tolerance = goal->path_tolerance[i].position;
            if(tolerance == 0)
                path_tolerance_[i] = default_path_tolerance_[i];
            else if(tolerance < 0)
                path_tolerance_[i] = 2*M_PI;
            else
                path_tolerance_[i] = tolerance;

        }
    }

    // === GET GOAL TOLERANCE ===
    if(goal->goal_tolerance.size() != ykin::N_JOINTS)
        goal_tolerance_ = default_goal_tolerance_;
    else
    {
        for(uint i=0; i<ykin::N_JOINTS; i++)
        {
            // goal tolerance
            double tolerance = goal->goal_tolerance[i].position;
            if(tolerance == 0)
                goal_tolerance_[i] = default_goal_tolerance_[i];
            else if(tolerance < 0)
                goal_tolerance_[i] = 2*M_PI;
            else
                goal_tolerance_[i] = tolerance;
        }
    }
    goal_time_tolerance_ = goal->goal_time_tolerance.toSec();
    feedback_.header = goal->trajectory.header;
    feedback_.joint_names = goal->trajectory.joint_names;
    last_point_ = trajectory_.begin();
    next_point_ = last_point_ + 1;
    has_active_goal_ = true;
    trajectory_ended_ = false;

    ROS_INFO("Received trajectory with %d valid points. Starting movement...", (int)trajectory_.size());
}
