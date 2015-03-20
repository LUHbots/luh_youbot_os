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


#include "luh_youbot_manipulation/module_base_controller/base_controller.h"


//########## CONSTRUCTOR ###############################################################################################
ModuleBaseController::ModuleBaseController(): ManipulationModule(),
    align_base_server_(NULL),
    move_base_server_(NULL),
    approach_server_(NULL)
{    
}

//########## DESTRUCTOR ################################################################################################
ModuleBaseController::~ModuleBaseController()
{
    delete align_base_server_;
    delete move_base_server_;
    delete approach_server_;
}

//########## INITIALIZATION ############################################################################################
void ModuleBaseController::init()
{
    ROS_INFO("Initialising Base Controller Module...");

    status_ = STATUS_IDLE;

    // === PARAMETERS ===
    activated_ = true;
    mode_ = IDLE;
    node_->param("module_base_controller/max_velocity_x", max_velocity_x_, 0.2);
    node_->param("module_base_controller/max_velocity_y", max_velocity_y_, 0.2);
    node_->param("module_base_controller/max_velocity_theta", max_velocity_theta_, 0.4);
    node_->param("module_base_controller/velocity_p_factor_x", velocity_p_factor_x_, 2.0);
    node_->param("module_base_controller/velocity_p_factor_y", velocity_p_factor_y_, 2.0);
    node_->param("module_base_controller/velocity_p_factor_theta", velocity_p_factor_theta_, 0.7);
    node_->param("module_base_controller/velocity_i_factor_x", velocity_i_factor_x_, 0.1);
    node_->param("module_base_controller/velocity_i_factor_y", velocity_i_factor_y_, 0.1);
    node_->param("module_base_controller/velocity_i_factor_theta", velocity_i_factor_theta_, 0.1);
    node_->param("module_base_controller/velocity_d_factor_x", velocity_d_factor_x_, 0.1);
    node_->param("module_base_controller/velocity_d_factor_y", velocity_d_factor_y_, 0.1);
    node_->param("module_base_controller/velocity_d_factor_theta", velocity_d_factor_theta_, 0.1);
    node_->param("module_base_controller/position_tolerance_x", position_tolerance_x_, 0.01);
    node_->param("module_base_controller/position_tolerance_y", position_tolerance_y_, 0.01);
    node_->param("module_base_controller/position_tolerance_theta", position_tolerance_theta_, 5.0);
    position_tolerance_theta_ *= M_PI / 180;
//    node_->param("module_base_controller/min_dist_x", min_dist_x_, 0.061);
//    node_->param("module_base_controller/min_dist_y", min_dist_y_, 0.185);
    node_->param("module_base_controller/velocity_command_timeout", velocity_command_timeout_, 1.0);
    node_->param("module_base_controller/memory_size", memory_size_, 5);
    node_->param("module_base_controller/align_stop_timeout", align_stop_timeout_, 1.0);
    node_->param("module_base_controller/align_fail_timeout", align_fail_timeout_, 1.0);
    node_->param("module_base_controller/align_topic", align_topic_, std::string(""));

    // === SUBSCRIBERS ===
//    laser_listener_ = node_->subscribe("/scan", 10, &ModuleBaseController::laserCallback, this);
    velocity_subscriber_ = node_->subscribe("/cmd_vel", 10, &ModuleBaseController::velocityCallback, this);
    if(!align_topic_.empty())
        pose_subscriber_ = node_->subscribe(align_topic_, 1, &ModuleBaseController::poseCallback, this);
    else
        ROS_INFO("No pose topic for alignment action specified.");

    laser_subscriber_ = node_->subscribe("laser_watchdog/distances", 1, &ModuleBaseController::laserCallback, this);

    // === ACTION SERVERS ===
    move_base_server_ = new MoveBaseServer(*node_, "youbot_base/move", false);
    move_base_server_->registerGoalCallback(boost::bind(&ModuleBaseController::moveBaseCallback, this));
    move_base_server_->registerPreemptCallback(boost::bind(&ModuleBaseController::preemptCallback, this));
    move_base_server_->start();   
    align_base_server_ = new AlignBaseServer(*node_, "youbot_base/align", false);
    align_base_server_->registerGoalCallback(boost::bind(&ModuleBaseController::alignBaseCallback, this));
    align_base_server_->registerPreemptCallback(boost::bind(&ModuleBaseController::preemptCallback, this));
    align_base_server_->start();
    approach_server_ = new ApproachServer(*node_, "youbot_base/approach", false);
    approach_server_->registerGoalCallback(boost::bind(&ModuleBaseController::approachCallback, this));
    approach_server_->registerPreemptCallback(boost::bind(&ModuleBaseController::preemptCallback, this));
    approach_server_->start();

    // === SERVICE SERVER ===
    stop_server_ = node_->advertiseService("youbot_base/stop", &ModuleBaseController::stopCallback, this);
    get_pose_server_ = node_->advertiseService("youbot_base/get_pose", &ModuleBaseController::getPoseCallback, this);

    // === PUBLISHER ===
    error_publisher_ = node_->advertise<geometry_msgs::Pose2D>("youbot_base/position_error", 100);


    ROS_INFO("Base Controller Module initialised.");
}

//########## UPDATE ####################################################################################################
void ModuleBaseController::update()
{
    if(!activated_)
        return;

    updateMovedDistance();

    if(mode_ == POSITION)
        updatePositionMode();

    else if(mode_ == VELOCITY)
        updateVelocityMode();

    else if(mode_ == ALIGN)
        updateAlignMode();

    else if(mode_ == APPROACH)
        updateApproachMode();
    else
        return;

    // publish velocity
    youbot_->base()->setVelocity(velocity_command_);
}

//########## UPDATE MOVED DISTANCE #####################################################################################
void ModuleBaseController::updateMovedDistance()
{
    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

    // get difference between current pose and goal pose
    double diff_x = current_pose_.x - start_pose_.x;
    double diff_y = current_pose_.y - start_pose_.y;
    double diff_theta = current_pose_.theta - start_pose_.theta;
    if(diff_theta > M_PI)
        diff_theta -= 2*M_PI;
    else if(diff_theta < -M_PI)
        diff_theta += 2*M_PI;

    double diff_x_t = diff_x * cos(base_pose.theta) + diff_y * sin(base_pose.theta);
    double diff_y_t = diff_y * cos(base_pose.theta) - diff_x * sin(base_pose.theta);

    moved_distance_.x = diff_x_t;
    moved_distance_.y = diff_y_t;
    moved_distance_.theta = diff_theta;
}

//########## UPDATE POSITION MODE ######################################################################################
void ModuleBaseController::updatePositionMode()
{
    double diff_x_t = goal_pose_.x - moved_distance_.x;
    double diff_y_t = goal_pose_.y - moved_distance_.y;
    double diff_theta = goal_pose_.theta - moved_distance_.theta;

    if(diff_theta > M_PI)
        diff_theta -= 2*M_PI;
    else if(diff_theta < -M_PI)
        diff_theta += 2*M_PI;

    bool goal_reached = true;

    // check x
    if(std::fabs(diff_x_t) > position_tolerance_x_)
    {
        goal_reached = false;
        velocity_command_.linear.x = diff_x_t * velocity_p_factor_x_;

        velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
        velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);
    }
    else
        velocity_command_.linear.x = 0;

    // check y
    if(std::fabs(diff_y_t) > position_tolerance_y_)
    {
        goal_reached = false;
        velocity_command_.linear.y = diff_y_t * velocity_p_factor_y_;

        velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
        velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);
    }
    else
        velocity_command_.linear.y = 0;

    // check theta
    if(std::fabs(diff_theta) > position_tolerance_theta_)
    {
        goal_reached = false;
        velocity_command_.angular.z = diff_theta * velocity_p_factor_theta_;

        velocity_command_.angular.z = std::min(velocity_command_.angular.z, max_velocity_theta_);
        velocity_command_.angular.z = std::max(velocity_command_.angular.z, -max_velocity_theta_);
    }
    else
        velocity_command_.angular.z = 0;


    // if goal is reached: action succeeded
    if(goal_reached)
    {
        mode_ = IDLE;
        base_is_busy_ = false;
        velocity_command_.linear.x = 0;
        velocity_command_.linear.y = 0;
        velocity_command_.angular.z = 0;

        ROS_INFO("Base movement finished.");
        move_base_server_->setSucceeded(moved_distance_);
    }
}

//########## UPDATE VELOCITY MODE ######################################################################################
void ModuleBaseController::updateVelocityMode()
{
    double time_passed = (ros::Time::now() - last_update_time_).toSec();

    if(time_passed > velocity_command_timeout_)
    {
        velocity_command_.linear.x = 0;
        velocity_command_.linear.y = 0;
        velocity_command_.angular.z = 0;

        mode_ = IDLE;
    }
}

//########## UPDATE ALIGN MODE #########################################################################################
void ModuleBaseController::updateAlignMode()
{
    // === CHECK TIMEOUT ===
    double delta_t = (ros::Time::now() - last_update_time_).toSec();

    if(delta_t > align_stop_timeout_)
    {
        if(!(velocity_command_.linear.x == 0 && velocity_command_.linear.y == 0 && velocity_command_.angular.z == 0))
        {
            ROS_WARN("No pose message received in %f seconds. Stopping.", align_stop_timeout_);
            velocity_command_.linear.x = 0;
            velocity_command_.linear.y = 0;
            velocity_command_.angular.z = 0;
        }
    }

    if(delta_t > align_fail_timeout_)
    {
        ROS_ERROR("No pose message received in %f seconds. Aborting.", align_fail_timeout_);

        mode_ = IDLE;
        base_is_busy_ = false;

        luh_youbot_msgs::AlignBaseToPoseResult result;
        result.moved_distance.x = moved_distance_.x;
        result.moved_distance.y = moved_distance_.y;
        result.moved_distance.theta = moved_distance_.theta;
        align_base_server_->setAborted(result);
    }

}

//########## UPDATE ALIGN MODE #########################################################################################
void ModuleBaseController::updateApproachMode()
{
    double diff_x_t, diff_y_t, diff_theta;

    if(approach_goal_.front > 0)
        diff_x_t = distances_.front - approach_goal_.front;
    else if(approach_goal_.back > 0)
        diff_x_t = approach_goal_.back - distances_.back;
    else
        diff_x_t = 0;

    if(approach_goal_.left > 0)
        diff_y_t = distances_.left - approach_goal_.left;
    else if(approach_goal_.right > 0)
        diff_y_t = approach_goal_.right - distances_.right;
    else
        diff_y_t = 0;

    diff_theta = 0;

    if(diff_theta > M_PI)
        diff_theta -= 2*M_PI;
    else if(diff_theta < -M_PI)
        diff_theta += 2*M_PI;

    bool goal_reached = true;

    // check x
    if(std::fabs(diff_x_t) > position_tolerance_x_)
    {
        goal_reached = false;
        velocity_command_.linear.x = diff_x_t * velocity_p_factor_x_;

        velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
        velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);
    }
    else
        velocity_command_.linear.x = 0;

    // check y
    if(std::fabs(diff_y_t) > position_tolerance_y_)
    {
        goal_reached = false;
        velocity_command_.linear.y = diff_y_t * velocity_p_factor_y_;

        velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
        velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);
    }
    else
        velocity_command_.linear.y = 0;

    // check theta
    if(std::fabs(diff_theta) > position_tolerance_theta_)
    {
        goal_reached = false;
        velocity_command_.angular.z = diff_theta * velocity_p_factor_theta_;

        velocity_command_.angular.z = std::min(velocity_command_.angular.z, max_velocity_theta_);
        velocity_command_.angular.z = std::max(velocity_command_.angular.z, -max_velocity_theta_);
    }
    else
        velocity_command_.angular.z = 0;

    if(goal_reached)
    {
        mode_ = IDLE;
        base_is_busy_ = false;
        velocity_command_.linear.x = 0;
        velocity_command_.linear.y = 0;
        velocity_command_.angular.z = 0;

        ROS_INFO("Base movement finished.");
        luh_youbot_msgs::ApproachBaseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        approach_server_->setSucceeded(res);
    }
}

//########## ACTIVATE ##################################################################################################
void ModuleBaseController::activate()
{
    activated_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleBaseController::deactivate()
{
    activated_ = false;
}

//########## EMERGENCY STOP ############################################################################################
void ModuleBaseController::emergencyStop()
{
    velocity_command_.linear.x = 0;
    velocity_command_.linear.y = 0;
    velocity_command_.angular.z = 0;

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
        base_is_busy_ = false;

    mode_ = IDLE;

    youbot_->base()->setVelocity(velocity_command_);

    if(move_base_server_->isActive())
        move_base_server_->setAborted(moved_distance_);
    if(align_base_server_->isActive())
    {
        luh_youbot_msgs::AlignBaseToPoseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        align_base_server_->setAborted(res);
    }
    if(approach_server_->isActive())
    {
        luh_youbot_msgs::ApproachBaseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        approach_server_->setPreempted(res);
    }
}

//###################### CALLBACK: PREEMPTION ##########################################################################
void ModuleBaseController::preemptCallback()
{
    velocity_command_.linear.x = 0;
    velocity_command_.linear.y = 0;
    velocity_command_.angular.z = 0;

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
        base_is_busy_ = false;

    mode_ = IDLE;

    youbot_->base()->setVelocity(velocity_command_);

    if(move_base_server_->isActive())
        move_base_server_->setPreempted(moved_distance_);
    if(align_base_server_->isActive())
    {
        luh_youbot_msgs::AlignBaseToPoseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        align_base_server_->setPreempted(res);
    }
    if(approach_server_->isActive())
    {
        luh_youbot_msgs::ApproachBaseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        approach_server_->setPreempted(res);
    }
}

//###################### CALLBACK: VELOCITY ############################################################################
void ModuleBaseController::velocityCallback(const geometry_msgs::Twist::ConstPtr &velocity_msg)
{
    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    {
        preemptCallback();
    }

    velocity_command_ = *velocity_msg;

    velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
    velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);

    velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
    velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);

    velocity_command_.angular.z = std::min(velocity_command_.angular.z, max_velocity_theta_);
    velocity_command_.angular.z = std::max(velocity_command_.angular.z, -max_velocity_theta_);

    last_update_time_ = ros::Time::now();

    mode_ = VELOCITY;
}

//###################### CALLBACK: MOVE BASE ###########################################################################
void ModuleBaseController::moveBaseCallback()
{    
    ROS_INFO("==== Module Base Controller ====");

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    {
        preemptCallback();
    }

    goal_pose_ = *(move_base_server_->acceptNewGoal());

    print_counter_ = 0;
    ROS_INFO("Request to move x=%f; y=%f; theta=%f.", goal_pose_.x, goal_pose_.y, goal_pose_.theta);

    if(!activated_ || base_is_busy_)
    {
        if(base_is_busy_)
            ROS_ERROR("Base is busy. Can't accept new goals.");
        else
            ROS_ERROR("Module is deactivated. Can't accept new goals.");

        move_base_server_->setAborted();
        return;
    }

    ROS_INFO("Moving base...");

    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

    moved_distance_.x = 0;
    moved_distance_.y = 0;
    moved_distance_.theta = 0;

    start_pose_ = current_pose_;
    mode_ = POSITION;
    base_is_busy_ = true;
}

//###################### CALLBACK: ALIGN BASE ##########################################################################
void ModuleBaseController::alignBaseCallback()
{
    ROS_INFO("==== Module Base Controller ====");

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    {
        preemptCallback();
    }

    align_goal_ = *(align_base_server_->acceptNewGoal());

    // debug output
    ROS_INFO("Goal position in %s: [%f | %f]", align_goal_.frame_id.c_str(), align_goal_.x_goal, align_goal_.y_goal);

    if(!activated_ || base_is_busy_ || align_topic_.empty())
    {
        if(base_is_busy_)
            ROS_ERROR("Base is busy. Can't accept new goals.");
        else if(!activated_)
            ROS_ERROR("Module is deactivated. Can't accept new goals.");
        else
            ROS_ERROR("No Pose topic specified.");

        align_base_server_->setAborted();
        return;
    }

    // transform goal pose
    try
    {
        tf_listener_->lookupTransform("base_link", align_goal_.frame_id, ros::Time(0), align_pose_transform_);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        align_base_server_->setAborted();
    }

    tf::Vector3 goal_pos(align_goal_.x_goal, align_goal_.y_goal, 0.0);
    tf::Vector3 pos_trans = align_pose_transform_(goal_pos);
    align_goal_.x_goal = pos_trans.x();
    align_goal_.y_goal = pos_trans.y();
    align_goal_.frame_id = "base_link";

    // debug output
    ROS_INFO("Goal position in %s: [%f | %f]", align_goal_.frame_id.c_str(), align_goal_.x_goal, align_goal_.y_goal);

    // subscriber to pose topic


    ROS_INFO("Aligning base...");

    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

    moved_distance_.x = 0;
    moved_distance_.y = 0;
    moved_distance_.theta = 0;

    // reset memory
    error_memory_.resize(memory_size_);
    time_memory_.assign(memory_size_, 0);

    memory_index_ = 0;

    accumulated_error_.x = 0;
    accumulated_error_.y = 0;
    accumulated_error_.theta = 0;

    BasePose zero_pose;
    zero_pose.x = 0;
    zero_pose.y = 0;
    zero_pose.theta = 0;
    last_two_errors_.push(zero_pose);
    last_two_errors_.push(zero_pose);

    last_update_time_ = ros::Time::now();

    start_pose_ = current_pose_;
    mode_ = ALIGN;
    base_is_busy_ = true;
}

//###################### CALLBACK: POSE ################################################################################
void ModuleBaseController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    if(mode_ != ALIGN)
        return;

    // === TRANSFORM POSE ===
    tf::Vector3 pos(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
    tf::Vector3 pos_trans = align_pose_transform_(pos);

    // === GET VELOCTY ===

    // pose difference
    double diff_x = pos_trans.x() - align_goal_.x_goal;
    double diff_y = pos_trans.y() - align_goal_.y_goal;

    //debug output
    ROS_INFO("error x: %f - %f = %f", pos_trans.x(), align_goal_.x_goal, diff_x);
    ROS_INFO("error y: %f - %f = %f", pos_trans.y(), align_goal_.y_goal, diff_y);

    // publish error
    geometry_msgs::Pose2D error_msg;
    error_msg.x = diff_x;
    error_msg.y = diff_y;
    error_publisher_.publish(error_msg);

    // check time diff for zeros
    bool has_zeros = false;
    for(uint i=0; i<time_memory_.size(); i++)
    {
        if(time_memory_[i] == 0)
        {
            has_zeros = true;
            break;
        }
    }

    // if all time diffs are valid: check velocity
    if(!has_zeros)
    {
        double time = 0;
        BasePose pos;
        pos.x = 0;
        pos.y = 0;
        pos.theta = 0;
        for(int i=0; i<memory_size_; i++)
        {
            time += time_memory_[i];
            pos.x += error_memory_[i].x;
            pos.y += error_memory_[i].y;
        }

        double velocity_x = pos.x / time;
        double velocity_y = pos.y / time;

        // debug output
        ROS_INFO("velocity x: %f / %f = %f", pos.x, time, velocity_x);
        ROS_INFO("velocity y: %f / %f = %f", pos.y, time, velocity_y);
        ROS_INFO("----------");

        // === CHECK TOLERANCES ===
        if(fabs(velocity_x) < align_goal_.x_vel_tolerance
                && fabs(velocity_y) < align_goal_.y_vel_tolerance
                && fabs(diff_x) < align_goal_.x_pos_tolerance
                && fabs(diff_y) < align_goal_.y_pos_tolerance)
        {
            ROS_INFO("Alignment succeeded.");

            luh_youbot_msgs::AlignBaseToPoseResult result;
            result.moved_distance.x = moved_distance_.x;
            result.moved_distance.y = moved_distance_.y;
            result.moved_distance.theta = moved_distance_.theta;
            result.last_pose = *pose_msg;

            mode_ = IDLE;
            align_base_server_->setSucceeded(result);
            return;
        }

    }


    // === PID CONTROLLER ===

    // passed time
    ros::Time current_time = ros::Time::now();
    double delta_t = (current_time - last_update_time_).toSec();
    last_update_time_ = current_time;

    // PID controller
    velocity_command_.linear.x += (velocity_p_factor_x_ + velocity_d_factor_x_/delta_t) * diff_x
            + (velocity_i_factor_x_ * delta_t - velocity_p_factor_x_ - 2 * velocity_d_factor_x_ / delta_t)
            * last_two_errors_.back().x
            + velocity_d_factor_x_ / delta_t * last_two_errors_.front().x;

    velocity_command_.linear.y += (velocity_p_factor_y_ + velocity_d_factor_y_/delta_t) * diff_y
            + (velocity_i_factor_y_ * delta_t - velocity_p_factor_y_ - 2 * velocity_d_factor_y_ / delta_t)
            * last_two_errors_.back().y
            + velocity_d_factor_y_ / delta_t * last_two_errors_.front().y;

    // saturation
    velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
    velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);

    velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
    velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);


    // === UPDATE MEMORY ===
    BasePose current_error;
    current_error.theta = 0;
    current_error.x = diff_x;
    current_error.y = diff_y;
    last_two_errors_.push(current_error);
    last_two_errors_.pop();

    BasePose error_diff;
    error_diff.x = current_error.x - last_two_errors_.front().x;
    error_diff.y = current_error.y - last_two_errors_.front().y;
    error_diff.theta = current_error.theta - last_two_errors_.front().theta;

    time_memory_[memory_index_] = delta_t;
    error_memory_[memory_index_] = error_diff;
    memory_index_ = (memory_index_ + 1) % memory_size_;

    // === UPDATE MOVED DISTANCE ===
    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

    // get difference between current pose and start pose
    double dx = current_pose_.x - start_pose_.x;
    double dy = current_pose_.y - start_pose_.y;
    double dtheta = current_pose_.theta - start_pose_.theta;

    double dx_t = dx * cos(base_pose.theta) + dy * sin(base_pose.theta);
    double dy_t = dy * cos(base_pose.theta) - dx * sin(base_pose.theta);

    moved_distance_.x = dx_t;
    moved_distance_.y = dy_t;
    moved_distance_.theta = dtheta;
}

//###################### CALLBACK: STOP ################################################################################
bool ModuleBaseController::stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    preemptCallback();

    return true;
}

//###################### CALLBACK: LASER WATCHDOG ######################################################################
void ModuleBaseController::laserCallback(const luh_laser_watchdog::Distances::ConstPtr &distances)
{
    distances_ = *distances;
}


//###################### CALLBACK: APPROACH ACTION #####################################################################
void ModuleBaseController::approachCallback()
{
    ROS_INFO("==== Module Base Controller ====");

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    {
        preemptCallback();
    }

    approach_goal_ = *(approach_server_->acceptNewGoal());

    if(!activated_ || base_is_busy_ || laser_subscriber_.getNumPublishers() == 0)
    {
        if(base_is_busy_)
            ROS_ERROR("Base is busy. Can't accept new goals.");
        else if(!activated_)
            ROS_ERROR("Module is deactivated. Can't accept new goals.");
        else
            ROS_ERROR("No laser scanner data available. Can't accept new goals.");

        move_base_server_->setAborted();
        return;
    }

    ROS_INFO("Moving base...");

    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

    moved_distance_.x = 0;
    moved_distance_.y = 0;
    moved_distance_.theta = 0;

    start_pose_ = current_pose_;
    mode_ = APPROACH;
    base_is_busy_ = true;
}

//###################### CALLBACK: GET BASE POSE #######################################################################
bool ModuleBaseController::getPoseCallback(luh_youbot_msgs::GetBasePose::Request &req,
                                           luh_youbot_msgs::GetBasePose::Response &res)
{
    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();

    res.x = base_pose.x;
    res.y = base_pose.y;
    res.theta = base_pose.theta;

    return true;
}
