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

#include "luh_youbot_controller/module_interpolation/module_interpolation.h"
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Float64MultiArray.h>
#include <luh_youbot_msgs/interpolation_mode.h>
#include <std_msgs/Float64.h>

namespace ykin = luh_youbot_kinematics;

//###################### CONSTRUCTOR ###################################################################################
ModuleInterpolation::ModuleInterpolation(): ControllerModule(),
    cartesian_pose_server_(NULL),
    cylindric_pose_server_(NULL),
    joint_pose_server_(NULL),
    named_pose_server_(NULL),
    torque_controller_(NULL),
    youbot_dynamics_(NULL)
{

}

//###################### DESTRUCTOR ####################################################################################
ModuleInterpolation::~ModuleInterpolation()
{
    delete cartesian_pose_server_;
    delete cylindric_pose_server_;
    delete joint_pose_server_;
    delete named_pose_server_;
    delete torque_controller_;
    delete youbot_dynamics_;
}

//###################### INIT ##########################################################################################
void ModuleInterpolation::init()
{
    ROS_INFO("MI: Initialising Interpolation Module...");

    // === INITIALISATION ===
    this->internal_state_ = INACTIVE;
    this->got_jointstate_ = false;
    this->got_command_ = false;
    this->moving_ = false;
    this->timeout_time_ = 0;
    //    this->stop_count_ = 0;
    //    this->deactivated_ = false;
    //    this->preempted_ = false;
    this->mode_changed_ = true;
    controller_test_mode_ = false;

    // === PARAMETERS ===
    this->node_->param("module_interpolation/timeout_threshold", this->timeout_threshold_, 1.0);
    std::vector<double> controller_kp(5, 0.0);
    std::vector<double> controller_ki(5, 0.0);
    std::vector<double> controller_kd(5, 0.0);
    std::vector<double> controller_i_min(5, 0.0);
    std::vector<double> controller_i_max(5, 0.0);
    ros::param::get("module_interpolation/controller_pid/kp", controller_kp);
    ros::param::get("module_interpolation/controller_pid/ki", controller_ki);
    ros::param::get("module_interpolation/controller_pid/kd", controller_kd);
    ros::param::get("module_interpolation/controller_pid/i_min", controller_i_min);
    ros::param::get("module_interpolation/controller_pid/i_max", controller_i_max);
    node_->param("module_interpolation/controller_test_mode", controller_test_mode_, false);
    if(controller_test_mode_)
        timeout_threshold_ *= 5;
    std::string mode_string;
    node_->param("module_interpolation/command_mode", mode_string, std::string("POSITION"));

    double max_vel_r, max_vel_q1, max_vel_q2, max_vel_q3, max_vel_q4, max_vel_z, max_vel_theta, max_vel_q5;
    double max_vel_x, max_vel_y;
    double max_acc_r, max_acc_q1, max_acc_q2, max_acc_q3, max_acc_q4, max_acc_z, max_acc_theta, max_acc_q5;
    double max_acc_x, max_acc_y;

    node_->param("module_interpolation/max_vel_q1",    max_vel_q1,    2.00);
    node_->param("module_interpolation/max_vel_q2",    max_vel_q2,    2.00);
    node_->param("module_interpolation/max_vel_q3",    max_vel_q3,    2.00);
    node_->param("module_interpolation/max_vel_q4",    max_vel_q4,    2.00);
    node_->param("module_interpolation/max_vel_q5",    max_vel_q5,    2.00);
    node_->param("module_interpolation/max_vel_r",     max_vel_r,     0.1);
    node_->param("module_interpolation/max_vel_x",     max_vel_x,     0.1);
    node_->param("module_interpolation/max_vel_y",     max_vel_y,     0.1);
    node_->param("module_interpolation/max_vel_z",     max_vel_z,     0.1);
    node_->param("module_interpolation/max_vel_theta", max_vel_theta, 0.5);

    node_->param("module_interpolation/max_acc_q1",    max_acc_q1,    1.00);
    node_->param("module_interpolation/max_acc_q2",    max_acc_q2,    1.00);
    node_->param("module_interpolation/max_acc_q3",    max_acc_q3,    1.00);
    node_->param("module_interpolation/max_acc_q4",    max_acc_q4,    1.00);
    node_->param("module_interpolation/max_acc_q5",    max_acc_q5,    1.00);
    node_->param("module_interpolation/max_acc_r",     max_acc_r,     0.1);
    node_->param("module_interpolation/max_acc_x",     max_acc_x,     0.1);
    node_->param("module_interpolation/max_acc_y",     max_acc_y,     0.1);
    node_->param("module_interpolation/max_acc_z",     max_acc_z,     0.1);
    node_->param("module_interpolation/max_acc_theta", max_acc_theta, 0.50);


    if(mode_string.compare("VELOCITY") == 0)
    {
        ROS_INFO("MI: Running in VELOCITY CONTROL MODE");
        command_mode_ = VELOCITY;
    }
    else if(mode_string.compare("TORQUE") == 0)
    {
        ROS_INFO("MI: Running in TORQUE CONTROL MODE");
        command_mode_ = TORQUE;
    }
    else
    {
        ROS_INFO("MI: Running in POSITION CONTROL MODE");
        command_mode_ = POSITION;
    }

    // === SUBSCRIBERS ===
    this->cyl_velocity_subscriber_  =
            this->node_->subscribe("arm_1/cylindric_velocity", 1, &ModuleInterpolation::cylVelocityCallback, this);
    this->cart_velocity_subscriber_ =
            this->node_->subscribe("arm_1/cartesian_velocity", 1, &ModuleInterpolation::cartVelocityCallback, this);
    this->jnt_velocity_subscriber_  =
            this->node_->subscribe("arm_1/joint_velocity", 1, &ModuleInterpolation::jntVelocityCallback, this);

    // === SERVICE SERVERS ===
    set_cart_vel_server_ = this->node_->advertiseService("arm_1/set_cartesian_velocity",
                                                         &ModuleInterpolation::setCartesianVelocityCallback, this);
    set_cyl_vel_server_ = this->node_->advertiseService("arm_1/set_cylindric_velocity",
                                                        &ModuleInterpolation::setCylindricVelocityCallback, this);
    set_jnt_vel_server_ = this->node_->advertiseService("arm_1/set_joint_velocity",
                                                        &ModuleInterpolation::setJointVelocityCallback, this);

    // === ACTION SERVERS ===
    this->cartesian_pose_server_ = new CartesianServer(*this->node_, "arm_1/to_cartesian_pose/inter", false);
    this->cartesian_pose_server_->registerGoalCallback(
                boost::bind(&ModuleInterpolation::cartesianPoseCallback, this));
    this->cylindric_pose_server_ = new CylindricServer(*this->node_, "arm_1/to_cylindric_pose/inter", false);
    this->cylindric_pose_server_->registerGoalCallback(
                boost::bind(&ModuleInterpolation::cylindricPoseCallback, this));
    this->joint_pose_server_ = new JointServer(*this->node_, "arm_1/to_joint_pose/inter", false);
    this->joint_pose_server_->registerGoalCallback(boost::bind(&ModuleInterpolation::jointPoseCallback, this));

    this->named_pose_server_ = new NamedServer(*this->node_, "arm_1/to_named_pose/inter", false);
    this->named_pose_server_->registerGoalCallback(boost::bind(&ModuleInterpolation::namedPoseCallback, this));

    cartesian_pose_server_->registerPreemptCallback(boost::bind(&ModuleInterpolation::preemptCallback, this));
    cylindric_pose_server_->registerPreemptCallback(boost::bind(&ModuleInterpolation::preemptCallback, this));
    joint_pose_server_->registerPreemptCallback(boost::bind(&ModuleInterpolation::preemptCallback, this));
    named_pose_server_->registerPreemptCallback(boost::bind(&ModuleInterpolation::preemptCallback, this));

    cartesian_pose_server_->start();
    cylindric_pose_server_->start();
    joint_pose_server_->start();
    named_pose_server_->start();

    // === PUBLISHERS ===
    cyl_position_command_pub_ =
            node_->advertise<std_msgs::Float64MultiArray>("module_interpolation/cyl_pos_cmd", 10);
    cyl_position_state_pub_ =
            node_->advertise<std_msgs::Float64MultiArray>("module_interpolation/cyl_pos_state", 10);

    // === INIT VECTORS ===
    max_cyl_velocity_.setQ1(max_vel_q1);
    max_cyl_velocity_.setR(max_vel_r);
    max_cyl_velocity_.setZ(max_vel_z);
    max_cyl_velocity_.setTheta(max_vel_theta);
    max_cyl_velocity_.setQ5(max_vel_q5);
    this->cyl_ramp_generator_.setMaxVelocity(max_cyl_velocity_);

    max_cyl_acceleration_.setQ1(max_acc_q1);
    max_cyl_acceleration_.setR(max_acc_r);
    max_cyl_acceleration_.setZ(max_acc_z);
    max_cyl_acceleration_.setTheta(max_acc_theta);
    max_cyl_acceleration_.setQ5(max_acc_q5);
    this->cyl_ramp_generator_.setMaxAcceleration(max_cyl_acceleration_);

    max_cart_velocity_.setX(max_vel_x);
    max_cart_velocity_.setY(max_vel_y);
    max_cart_velocity_.setZ(max_vel_z);
    max_cart_velocity_.setTheta(max_vel_theta);
    max_cart_velocity_.setQ5(max_vel_q5);
    this->cart_ramp_generator_.setMaxVelocity(max_cart_velocity_);

    max_cart_acceleration_.setX(max_acc_x);
    max_cart_acceleration_.setY(max_acc_y);
    max_cart_acceleration_.setZ(max_acc_z);
    max_cart_acceleration_.setTheta(max_acc_theta);
    max_cart_acceleration_.setQ5(max_acc_q5);
    this->cart_ramp_generator_.setMaxAcceleration(max_cart_acceleration_);

//    max_joint_velocity_.assign(ykin::MAX_JNT_VELOCITIES,
//                               ykin::MAX_JNT_VELOCITIES + ykin::N_JOINTS);
    max_joint_velocity_.setQ1(max_vel_q1);
    max_joint_velocity_.setQ2(max_vel_q2);
    max_joint_velocity_.setQ3(max_vel_q3);
    max_joint_velocity_.setQ4(max_vel_q4);
    max_joint_velocity_.setQ5(max_vel_q5);
    this->jnt_ramp_generator_.setMaxVelocity(max_joint_velocity_);

//    max_joint_acceleration_.assign(ykin::MAX_JNT_ACCELERATIONS,
//                                   ykin::MAX_JNT_ACCELERATIONS + ykin::N_JOINTS);
    max_joint_acceleration_.setQ1(max_acc_q1);
    max_joint_acceleration_.setQ2(max_acc_q2);
    max_joint_acceleration_.setQ3(max_acc_q3);
    max_joint_acceleration_.setQ4(max_acc_q4);
    max_joint_acceleration_.setQ5(max_acc_q5);
    this->jnt_ramp_generator_.setMaxAcceleration(max_joint_acceleration_);

    // === PID CONTROLLER ===
    controller_pid_.resize(5);
    for(int i=0; i<5; i++)
        controller_pid_[i].initPid(controller_kp[i],
                                   controller_ki[i],
                                   controller_kd[i],
                                   controller_i_max[i],
                                   controller_i_min[i]);

    // === TORQUE CONTROLLER ===
    if(command_mode_ == TORQUE)
    {
        torque_controller_ = new nAxesControllerTorque(*node_);
        youbot_dynamics_ = new YoubotDynamics();
    }

    ROS_INFO("MI: Interpolation Module initialised.");
}

//###################### ACTIVATE ######################################################################################
void ModuleInterpolation::activate()
{
    //    this->deactivated_ = false;
    if(internal_state_ == BLOCKED)
        internal_state_ = INACTIVE;
}

//###################### DEACTIVATE ####################################################################################
void ModuleInterpolation::deactivate()
{
    //    this->deactivated_ = true;
    if(internal_state_ == ACTIVE_VELOCITY || internal_state_ == ACTIVE_POSITION)
        stop();

    internal_state_ = BLOCKED;
}

//###################### CALLBACK: SET CARTESIAN VELOCITY ##############################################################
bool ModuleInterpolation::setCartesianVelocityCallback(luh_youbot_msgs::SetCartesianVelocity::Request &req,
                                  luh_youbot_msgs::SetCartesianVelocity::Response &res)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ykin::CartesianVelocity vel;
    vel.setX(req.max_velocity.x);
    vel.setY(req.max_velocity.y);
    vel.setZ(req.max_velocity.z);
    vel.setTheta(req.max_velocity.theta);
    vel.setQ5(req.max_velocity.q5);

    max_cart_velocity_ = vel.abs();
    max_cart_velocity_.printValues("Max cartesian velocity set to");
    return true;
}

//###################### CALLBACK: SET CYLINDRIC VELOCITY ##############################################################
bool ModuleInterpolation::setCylindricVelocityCallback(luh_youbot_msgs::SetCylindricVelocity::Request &req,
                                  luh_youbot_msgs::SetCylindricVelocity::Response &res)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ykin::CylindricVelocity vel;
    vel.setQ1(req.max_velocity.q1);
    vel.setR(req.max_velocity.r);
    vel.setZ(req.max_velocity.z);
    vel.setTheta(req.max_velocity.theta);
    vel.setQ5(req.max_velocity.q5);

    max_cyl_velocity_ = vel.abs();
    max_cyl_velocity_.printValues("Max cylindrical velocity set to");
    return true;
}

//###################### CALLBACK: SET JOINT VELOCITY ##################################################################
bool ModuleInterpolation::setJointVelocityCallback(luh_youbot_msgs::SetJointVelocity::Request &req,
                              luh_youbot_msgs::SetJointVelocity::Response &res)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ykin::JointVelocity vel;
    vel.setQ1(req.max_velocity.q1);
    vel.setQ2(req.max_velocity.q2);
    vel.setQ3(req.max_velocity.q3);
    vel.setQ4(req.max_velocity.q4);
    vel.setQ5(req.max_velocity.q5);
    vel = vel.abs();

    // safety check
    for(uint i=0; i<ykin::N_JOINTS; i++)
    {
        if(std::fabs(vel[i]) > ykin::MAX_JNT_VELOCITIES[i])
        {
            ROS_WARN("Desired joint velocity is too high.");
            return false;
        }
    }

    max_joint_velocity_ = vel;
    max_joint_velocity_.printValues("Max joint velocity set to");
    return true;
}

//###################### CALLBACK: PREEMPT #############################################################################
void ModuleInterpolation::preemptCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ROS_WARN("MI: Preemption callback.");
    //    preempted_ = true;

    if(cartesian_pose_server_->isActive())
    {
        cartesian_pose_server_->setPreempted();
    }
    if(cylindric_pose_server_->isActive())
    {
        cylindric_pose_server_->setPreempted();
    }
    if(joint_pose_server_->isActive())
    {
        joint_pose_server_->setPreempted();
    }
    if(named_pose_server_->isActive())
    {
        named_pose_server_->setPreempted();
    }

    if(internal_state_ == ACTIVE_POSITION)
    {
        stop();
    }
}

//###################### EMERGENCY STOP ################################################################################
void ModuleInterpolation::emergencyStop()
{
    if(cartesian_pose_server_->isActive())
    {
        cartesian_pose_server_->setAborted();
    }
    if(cylindric_pose_server_->isActive())
    {
        cylindric_pose_server_->setAborted();
    }
    if(joint_pose_server_->isActive())
    {
        joint_pose_server_->setAborted();
    }
    if(named_pose_server_->isActive())
    {
        named_pose_server_->setAborted();
    }

    // === SET FLAGS/STATE ===
    internal_state_ = INACTIVE;
    status_ = STATUS_IDLE;
    arm_is_busy_ = false;
    moving_ = false;

    // === RESET RAMP GENERATORS ===
    joint_state_position_ = youbot_->arm()->getJointPosition();
    ykin::GenericVector zero_velocity;
    ykin::CylindricPosition current_cyl_position = joint_state_position_.toCylindric();
    ykin::GenericVector current_cart_position = current_cyl_position.toCartesian();
    cart_ramp_generator_.resetCurrentState(zero_velocity, current_cart_position);
    cyl_ramp_generator_.resetCurrentState(zero_velocity, current_cyl_position);
    jnt_ramp_generator_.resetCurrentState(zero_velocity, joint_state_position_);
    cart_ramp_generator_.setTargetVelocity(zero_velocity);
    cyl_ramp_generator_.setTargetVelocity(zero_velocity);
    jnt_ramp_generator_.setTargetVelocity(zero_velocity);

    // === RESET TORQUE CONTROLLER ===
    if(command_mode_ == TORQUE)
        torque_controller_->reset();

    // === RESET LAST VELOCITY ===
    last_velocity_ = ykin::JointVelocity();
}

//###################### CHECK STATE: VELOCITY #########################################################################
bool ModuleInterpolation::checkStateVelocity()
{
    // === UPDATE JOINT STATE ===
    joint_state_position_ = youbot_->arm()->getJointPosition();
    joint_state_velocity_ = youbot_->arm()->getJointVelocity();

    if(joint_state_position_.empty())
        return false;

    switch(internal_state_)
    {
    case BLOCKED:
    {
        return false;
    }
    case INACTIVE:
    {
        return !arm_is_busy_;
    }
    case ACTIVE_VELOCITY:
    {
        if(arm_is_busy_)
        {
            internal_state_ = INACTIVE;
            return false;
        }
        return true;
    }
    case ACTIVE_POSITION:
    {
        return false;
    }
    }

    return false;
}

//###################### CHECK STATE: POSITION #########################################################################
bool ModuleInterpolation::checkStatePosition()
{
    // === UPDATE JOINT STATE ===
    joint_state_position_ = youbot_->arm()->getJointPosition();
    joint_state_velocity_ = youbot_->arm()->getJointVelocity();

    if(joint_state_position_.empty())
        return false;

    switch(internal_state_)
    {
    case BLOCKED:
    {
        return false;
    }
    case INACTIVE:
    {
        return !arm_is_busy_;
    }
    case ACTIVE_VELOCITY:
    {
        return true;
    }
    case ACTIVE_POSITION:
    {
        if(cartesian_pose_server_->isActive())
            cartesian_pose_server_->setPreempted();
        if(cylindric_pose_server_->isActive())
            cylindric_pose_server_->setPreempted();
        if(joint_pose_server_->isActive())
            joint_pose_server_->setPreempted();
        if(named_pose_server_->isActive())
            named_pose_server_->setPreempted();
        return true;
    }
    }

    return false;
}

//###################### CALLBACK: CARTESIAN VELOCITY ##################################################################
void ModuleInterpolation::cartVelocityCallback(const luh_youbot_msgs::CartesianVector::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!checkStateVelocity())
        return;

    if(coordinate_mode_ != CARTESIAN)
    {
        coordinate_mode_ = CARTESIAN;
        mode_changed_ = true;
    }
    else
        mode_changed_ = false;

    ykin::CartesianVelocity velocity;
    try
    {
        velocity = ykin::CartesianVelocity::fromMsg(*msg, tf_listener_);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("MI: %s",ex.what());
        return;
    }

    setVelocityCommand(velocity);
}

//###################### CALLBACK: JOINT VELOCITY ######################################################################
void ModuleInterpolation::jntVelocityCallback(const luh_youbot_msgs::JointVector::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!checkStateVelocity())
        return;

    if(coordinate_mode_ != JOINTSPACE)
    {
        coordinate_mode_ = JOINTSPACE;
        mode_changed_ = true;
    }
    else
        mode_changed_ = false;

    ykin::JointVelocity velocity;
    velocity.setQ1(msg->q1);
    velocity.setQ2(msg->q2);
    velocity.setQ3(msg->q3);
    velocity.setQ4(msg->q4);
    velocity.setQ5(msg->q5);

    setVelocityCommand(velocity);
}

//###################### CALLBACK: CYLINDRICAL VELOCITY ################################################################
void ModuleInterpolation::cylVelocityCallback(const luh_youbot_msgs::CylindricVector::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!checkStateVelocity())
        return;

    if(coordinate_mode_ != CYLINDRIC)
    {
        coordinate_mode_ = CYLINDRIC;
        mode_changed_ = true;
    }
    else
        mode_changed_ = false;

    ykin::CylindricVelocity velocity;
    velocity.setQ1(msg->q1);
    velocity.setR(msg->r);
    velocity.setZ(msg->z);
    velocity.setTheta(msg->theta);
    velocity.setQ5(msg->q5);

    setVelocityCommand(velocity);
}

//###################### SET VELOCITY COMMAND ##########################################################################
void ModuleInterpolation::setVelocityCommand(const ykin::GenericVector& velocity)
{
    // === SET VELOCITY COMMAND ===
    if(coordinate_mode_ == CARTESIAN)
    {
        // set max velocity and acceleration at current state
        ykin::CylindricPosition cyl_pos = joint_state_position_.toCylindric();
        ykin::CylindricVelocity cyl_vel = joint_state_velocity_.toCylindric(joint_state_position_);
        cart_ramp_generator_.setMaxAcceleration(max_cyl_acceleration_.toCartesian(cyl_vel, cyl_pos).abs());
        cart_ramp_generator_.setMaxVelocity(max_cyl_velocity_.toCartesian(cyl_pos).abs());
        cart_ramp_generator_.setTargetVelocity(velocity);
    }
    else if(coordinate_mode_ == CYLINDRIC)
        cyl_ramp_generator_.setTargetVelocity(velocity);
    else if(coordinate_mode_ == JOINTSPACE)
        jnt_ramp_generator_.setTargetVelocity(velocity);
    else
        return;

    // === SET FLAGS ===
    got_command_ = true;
    //     preempted_ = false;

    // === START ===
    if((!moving_ && !velocity.isZero()) || mode_changed_)
    {
        // update current state before starting
        ykin::CylindricPosition cyl_pos = joint_state_position_.toCylindric();
        ykin::CylindricVelocity cyl_vel = joint_state_velocity_.toCylindric(joint_state_position_);
        ykin::CartesianPosition car_pos = cyl_pos.toCartesian();
        ykin::CartesianVelocity car_vel = cyl_vel.toCartesian(cyl_pos);
        cyl_ramp_generator_.resetCurrentState(cyl_vel, cyl_pos);
        cart_ramp_generator_.resetCurrentState(car_vel, car_pos);
        jnt_ramp_generator_.resetCurrentState(joint_state_velocity_, joint_state_position_);

        // reset controller
        for(uint i=0; i<controller_pid_.size(); i++)
            controller_pid_[i].reset();
        if(command_mode_ == TORQUE)
            torque_controller_->reset();

        moving_ = true;
        status_ = STATUS_ACTIVE;
        internal_state_ = ACTIVE_VELOCITY;
    }

    // === RESET TIMEOUT TIMER ===
    t_now_ = ros::Time::now().toSec();
    if(!velocity.isZero())
        timeout_time_ = 0;


}

//###################### CALLBACK: CARTESIAN POSITION ##################################################################
void ModuleInterpolation::cartesianPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!checkStatePosition())
    {
        ROS_WARN("MI: Can't accept action goals.");
        cartesian_pose_server_->setAborted();
        return;
    }
    boost::shared_ptr<const luh_youbot_msgs::MoveToCartesianPoseGoal> goal = cartesian_pose_server_->acceptNewGoal();

    coordinate_mode_ = CARTESIAN;

    ykin::CartesianPosition position;

    try
    {
        position = ykin::CartesianPosition::fromMsg(
                    goal->pose, goal->pose_is_relative, joint_state_position_, tf_listener_);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("MI: %s",ex.what());
        cartesian_pose_server_->setAborted();
        return;
    }

    goal_position_ = position;

    goal_pose_is_named_ = false;

    setPositionCommand(position, true);
}

//###################### CALLBACK: CYLINDRIC POSITION ##################################################################
void ModuleInterpolation::cylindricPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!checkStatePosition())
    {
        ROS_WARN("MI: Can't accept action goals.");
        cylindric_pose_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::MoveToCylindricPoseGoal> goal = cylindric_pose_server_->acceptNewGoal();

    coordinate_mode_ = CYLINDRIC;

    ykin::CylindricPosition position_command;
    position_command.setQ1(goal->pose.q1);
    position_command.setR(goal->pose.r);
    position_command.setZ(goal->pose.z);
    position_command.setTheta(goal->pose.theta);
    position_command.setQ5(goal->pose.q5);

    goal_position_ = position_command;

    goal_pose_is_named_ = false;

    setPositionCommand(position_command, !goal->pose_is_relative);
}

//###################### CALLBACK: JOINT POSITION ######################################################################
void ModuleInterpolation::jointPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!checkStatePosition())
    {
        ROS_WARN("MI: Can't accept action goals.");
        cylindric_pose_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::MoveToJointPoseGoal> goal = joint_pose_server_->acceptNewGoal();

    coordinate_mode_ = JOINTSPACE;

    ykin::JointPosition position;
    position.setQ1(goal->pose.q1);
    position.setQ2(goal->pose.q2);
    position.setQ3(goal->pose.q3);
    position.setQ4(goal->pose.q4);
    position.setQ5(goal->pose.q5);

    goal_position_ = position;

    goal_pose_is_named_ = false;

    setPositionCommand(position, !goal->pose_is_relative);
}

//###################### CALLBACK: NAMED POSITION ######################################################################
void ModuleInterpolation::namedPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!checkStatePosition())
    {
        ROS_WARN("MI: Can't accept action goals.");
        named_pose_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::MoveToNamedPoseGoal> goal = named_pose_server_->acceptNewGoal();

    if(predefined_poses_.find(goal->pose_name) == predefined_poses_.end())
    {
        ROS_ERROR("Requested pose '%s' is unknown.", goal->pose_name.c_str());
        named_pose_server_->setAborted();
        return;
    }

    ykin::JointPosition joint_position = predefined_poses_[goal->pose_name];

    if(goal->interpolation_mode == luh_youbot_msgs::JOINTSPACE)
    {
        coordinate_mode_ = JOINTSPACE;
        goal_position_ = joint_position;
    }
    if(goal->interpolation_mode == luh_youbot_msgs::CYLINDRIC)
    {
        coordinate_mode_ = CYLINDRIC;
        goal_position_ = joint_position.toCylindric();
    }
    if(goal->interpolation_mode == luh_youbot_msgs::CARTESIAN)
    {
        coordinate_mode_ = CARTESIAN;
        goal_position_ = joint_position.toCartesian();
    }

    goal_pose_is_named_ = true;

    setPositionCommand(goal_position_, true);
}

//###################### SET POSITION COMMAND ##########################################################################
void ModuleInterpolation::setPositionCommand(const ykin::GenericVector &position, bool is_absolute)
{
    ROS_INFO("==== Module Interpolation ====");

    // === RESET EVERYTHING ===
    if(!moving_)
    {
        // update current state before starting
        ykin::CylindricPosition cyl_pos = joint_state_position_.toCylindric();
        ykin::CylindricVelocity cyl_vel = joint_state_velocity_.toCylindric(joint_state_position_);
        ykin::CartesianPosition car_pos = cyl_pos.toCartesian();
        ykin::CartesianVelocity car_vel = cyl_vel.toCartesian(cyl_pos);
        cyl_ramp_generator_.resetCurrentState(cyl_vel, cyl_pos);
        cart_ramp_generator_.resetCurrentState(car_vel, car_pos);
        jnt_ramp_generator_.resetCurrentState(joint_state_velocity_, joint_state_position_);

        // reset controller
        for(uint i=0; i<controller_pid_.size(); i++)
            controller_pid_[i].reset();
        if(command_mode_ == TORQUE)
            torque_controller_->reset();

        moving_ = true;
        status_ = STATUS_ACTIVE;
    }

    // === SET TARGET POSITION ===
    if(coordinate_mode_ == CARTESIAN)
    {
        // set max velocity and acceleration at current state (todo: update during movement?)
        //        ykin::CylindricPosition cyl_pos = joint_state_position_.toCylindric();
        //        ykin::CylindricVelocity cyl_vel = joint_state_velocity_.toCylindric(joint_state_position_);
        //        cart_ramp_generator_.setMaxAcceleration(max_cyl_acceleration_.toCartesian(cyl_vel, cyl_pos).abs());
        //        cart_ramp_generator_.setMaxVelocity(max_cyl_velocity_.toCartesian(cyl_pos).abs());

        // set target position
        cart_ramp_generator_.setMaxVelocity(max_cart_velocity_);
        cart_ramp_generator_.setTargetPosition(position, is_absolute);
    }
    else if(coordinate_mode_ == CYLINDRIC)
    {
        cyl_ramp_generator_.setMaxVelocity(max_cyl_velocity_);
        cyl_ramp_generator_.setTargetPosition(position, is_absolute);
    }
    else if(coordinate_mode_ == JOINTSPACE)
    {
        jnt_ramp_generator_.setMaxVelocity(max_joint_velocity_);
        jnt_ramp_generator_.setTargetPosition(position, is_absolute);
    }

    // === SET FLAGS ===
    got_command_ = true;
    //    preempted_ = false;

    ROS_INFO("MI: Goal position set.");

    // === RESET TIMOUT TIMER ==
    t_now_ = ros::Time::now().toSec();
    timeout_time_ = -10;

    internal_state_ = ACTIVE_POSITION;
    arm_is_busy_ = true;
}

//###################### UPDATE ########################################################################################
void ModuleInterpolation::update()
{
    // === CHECK STATE ===
    if(internal_state_ == INACTIVE || internal_state_ == BLOCKED)
        return;

    if(internal_state_ == ACTIVE_VELOCITY && arm_is_busy_)
    {
        internal_state_ = INACTIVE;
        moving_ = false;
        return;
    }

    // === UPDATE JOINT STATE ===
    joint_state_position_ = youbot_->arm()->getJointPosition();
    joint_state_velocity_ = youbot_->arm()->getJointVelocity();

    // === UPDATE TIME ===
    double new_time = ros::Time::now().toSec();
    double dt = new_time - t_now_;
    t_now_ = new_time;
    timeout_time_ += dt;

    // === CHECK TIMEOUT ===
    if(timeout_time_ >= timeout_threshold_)
    {
        ROS_INFO("MI: Time out.");
        stop();

        if(cartesian_pose_server_->isActive())
            cartesian_pose_server_->setAborted();
        if(cylindric_pose_server_->isActive())
            cylindric_pose_server_->setAborted();
        if(joint_pose_server_->isActive())
            joint_pose_server_->setAborted();
        if(named_pose_server_->isActive())
            named_pose_server_->setAborted();

        return;
    }

    // === UPDATE POSITION AND VELOCITY ===
    bool position_is_reachable = true;
    ykin::GenericVector actual_position;
    ykin::GenericVector position_command;
    ykin::GenericVector velocity;
    if(coordinate_mode_ == CARTESIAN)
    {
        cart_ramp_generator_.update();

        if(cart_ramp_generator_.targetPositionReached())
        {
            ROS_INFO("MI: Goal position reached.");
            internal_state_ = ACTIVE_VELOCITY;
            arm_is_busy_ = false;
            timeout_time_ = 0;
            if(goal_pose_is_named_)
                named_pose_server_->setSucceeded();
            else
                cartesian_pose_server_->setSucceeded();
        }

        velocity = cart_ramp_generator_.getVelocity();
        position_command = cart_ramp_generator_.getPosition();
        actual_position = joint_state_position_.toCartesian();

        position_is_reachable = ((ykin::CartesianPosition)position_command).isReachable();
    }
    else if(coordinate_mode_ == CYLINDRIC)
    {
        cyl_ramp_generator_.update();

        if(cyl_ramp_generator_.targetPositionReached())
        {
            ROS_INFO("MI: Goal position reached.");
            internal_state_ = ACTIVE_VELOCITY;
            arm_is_busy_ = false;
            timeout_time_ = 0;
            if(goal_pose_is_named_)
                named_pose_server_->setSucceeded();
            else
                cylindric_pose_server_->setSucceeded();
        }

        velocity = cyl_ramp_generator_.getVelocity();
        position_command = cyl_ramp_generator_.getPosition();
        actual_position = joint_state_position_.toCylindric();

        position_is_reachable = ((ykin::CylindricPosition)position_command).isReachable();
    }
    else if(coordinate_mode_ == JOINTSPACE)
    {
        jnt_ramp_generator_.update();

        if(jnt_ramp_generator_.targetPositionReached())
        {
            ROS_INFO("MI: Goal position reached.");
            internal_state_ = ACTIVE_VELOCITY;
            arm_is_busy_ = false;
            timeout_time_ = 0;
            if(goal_pose_is_named_)
                named_pose_server_->setSucceeded();
            else
                joint_pose_server_->setSucceeded();
        }

        velocity = jnt_ramp_generator_.getVelocity();
        position_command = jnt_ramp_generator_.getPosition();
        actual_position = joint_state_position_;

        position_is_reachable = true;
    }
    else
    {
        stop();

        if(cartesian_pose_server_->isActive())
            cartesian_pose_server_->setAborted();
        if(cylindric_pose_server_->isActive())
            cylindric_pose_server_->setAborted();
        if(joint_pose_server_->isActive())
            joint_pose_server_->setAborted();
        if(named_pose_server_->isActive())
            named_pose_server_->setAborted();

        return;
    }


    // === CHECK IF POSITION IS VALID ===
    if(!position_is_reachable)
    {
        ROS_WARN("MI: Position is unreachable.");
        stop();

        if(cartesian_pose_server_->isActive())
            cartesian_pose_server_->setAborted();
        if(cylindric_pose_server_->isActive())
            cylindric_pose_server_->setAborted();
        if(joint_pose_server_->isActive())
            joint_pose_server_->setAborted();
        if(named_pose_server_->isActive())
            named_pose_server_->setAborted();

        return;
    }

    // === GET JOINTSPACE VELOCITY ===
    ykin::JointVelocity joint_velocity;
    if(coordinate_mode_ == CARTESIAN)
    
        joint_velocity = ((ykin::CartesianVelocity)velocity).toJointspace(joint_state_position_);
    else if(coordinate_mode_ == CYLINDRIC)
        joint_velocity = ((ykin::CylindricVelocity)velocity).toJointspace(joint_state_position_);
    else
        joint_velocity = velocity;


    // === GET JOINT POSITION ===

    ykin::JointPosition joint_position;
    if(coordinate_mode_ == CARTESIAN)
    {
        joint_position = ((ykin::CartesianPosition)position_command).toJointspace(joint_state_position_);
    }
    else if(coordinate_mode_ == CYLINDRIC)
    {
        joint_position = ((ykin::CylindricPosition)position_command).toJointspace(joint_state_position_);
    }
    else
    {
        joint_position = position_command;
    }

    // === PUBLISH POSITIONS FOR CONTROLLER DESIGN ===
    std_msgs::Float64MultiArray pos_cmd_msg;
    std_msgs::Float64MultiArray pos_state_msg;
    if(controller_test_mode_)
        pos_cmd_msg.data = goal_position_;
    else
        pos_cmd_msg.data = joint_position;
    pos_state_msg.data = joint_state_position_;
    ykin::CartesianPosition crt_pos = joint_position.toCartesian();
    pos_cmd_msg.data.push_back(crt_pos.x());
    pos_cmd_msg.data.push_back(crt_pos.y());
    pos_cmd_msg.data.push_back(crt_pos.z());
    pos_cmd_msg.data.push_back(crt_pos.theta());

    crt_pos = joint_state_position_.toCartesian();
    pos_state_msg.data.push_back(crt_pos.x());
    pos_state_msg.data.push_back(crt_pos.y());
    pos_state_msg.data.push_back(crt_pos.z());
    pos_state_msg.data.push_back(crt_pos.theta());

    pos_cmd_msg.data.push_back(t_now_);
    pos_state_msg.data.push_back(t_now_);
    cyl_position_command_pub_.publish(pos_cmd_msg);
    cyl_position_state_pub_.publish(pos_state_msg);

    // === SAFETY CHECK ===
    double safety_factor = getSafetyFactor(joint_velocity);
    if(safety_factor == 0)
    {
        stop();

        if(cartesian_pose_server_->isActive())
            cartesian_pose_server_->setAborted();
        if(cylindric_pose_server_->isActive())
            cylindric_pose_server_->setAborted();
        if(joint_pose_server_->isActive())
            joint_pose_server_->setAborted();
        if(named_pose_server_->isActive())
            named_pose_server_->setAborted();

        return;
    }
    else if(safety_factor < 0.999)
    {
        ROS_WARN("MI: Safety factor is %f", safety_factor);

        joint_velocity *= safety_factor;

        if(coordinate_mode_ == CARTESIAN)
        {
            cart_ramp_generator_.adjustVelocity(safety_factor);
            joint_position =
                    ((ykin::CartesianPosition)cart_ramp_generator_.getPosition()).toJointspace(joint_state_position_);
        }
        else if(coordinate_mode_ == CYLINDRIC)
        {
            cyl_ramp_generator_.adjustVelocity(safety_factor);
            joint_position =
                    ((ykin::CylindricPosition)cyl_ramp_generator_.getPosition()).toJointspace(joint_state_position_);
        }
        else
        {
            jnt_ramp_generator_.adjustVelocity(safety_factor);
            joint_position = jnt_ramp_generator_.getPosition();
        }
    }

    // === TORQUE CONTROL MODE ===
    ykin::JointVector torques;
    if(command_mode_ == TORQUE)
    {
        // get torques
        //        ykin::JointAcceleration joint_acceleration = (joint_velocity - last_velocity_) / dt;
        //        ykin::JointVector efforts = youbot_dynamics_->getEffort(joint_position, joint_velocity, joint_acceleration);
        //        torques = torque_controller_->getTorques(joint_position, joint_velocity,
        //                                                 joint_state_position_, joint_state_velocity_, efforts);
        // -> moved after safety check
    }

    // === VELOCITY CONTROL MODE ===
    else if(command_mode_ == VELOCITY)
    {
        if(controller_test_mode_)
        {
            joint_velocity = ykin::JointVelocity();
            joint_position = goal_position_;
        }

        ykin::JointPosition position_error = joint_position - joint_state_position_;
        ros::Duration dt_ros(dt);
        for(unsigned int i=0; i<ykin::N_JOINTS; i++)
        {
            joint_velocity[i] += controller_pid_[i].computeCommand(position_error[i], dt_ros);
        }
    }

    if(!joint_position.isValid())
    {
        ROS_WARN("MI: Invalid joint position.");

        stop();

        if(cartesian_pose_server_->isActive())
            cartesian_pose_server_->setAborted();
        if(cylindric_pose_server_->isActive())
            cylindric_pose_server_->setAborted();
        if(joint_pose_server_->isActive())
            joint_pose_server_->setAborted();
        if(named_pose_server_->isActive())
            named_pose_server_->setAborted();

        return;
    }

    // === POSITION CONTROL MODE ===
    if(command_mode_ == POSITION)
    {
        youbot_->arm()->setJointPositions(joint_position);
    }

    // === TORQUE CONTROL MODE ===
    else if(command_mode_ == TORQUE)
    {
        ykin::JointAcceleration joint_acceleration = (joint_velocity - last_velocity_) / dt;
        ykin::JointVector efforts = youbot_dynamics_->getEffort(joint_position, joint_velocity, joint_acceleration);
        torques = torque_controller_->getTorques(joint_position, joint_velocity,
                                                 joint_state_position_, joint_state_velocity_, efforts);
        youbot_->arm()->setJointTorques(torques);
    }

    // === VELOCITY CONTROL MODE ===
    else
    {
        youbot_->arm()->setJointVelocities(joint_velocity);
    }

    last_velocity_ = joint_velocity;
}


//###################### SAFETY CHECK ##################################################################################
double ModuleInterpolation::getSafetyFactor(ykin::JointVelocity &jnt_velocity)
{
    ykin::JointVelocity max_velocities;
    max_velocities.setValues(ykin::MAX_JNT_VELOCITIES);
    double factor = 1.0;

    // === GET SAFETY FACTOR ===
    //    (velocity must be low enough to stop before reaching end position)
    for(unsigned int i=0; i<5; i++)
    {
        if(fabs(jnt_velocity[i]) >= 0.001)
        {
            double delta_pos;
            if(joint_state_velocity_[i] > 0)
            {
                delta_pos = ykin::MAX_JNT_POSITIONS[i] - joint_state_position_[i];
            }
            else
            {
                delta_pos = joint_state_position_[i] - ykin::MIN_JNT_POSITIONS[i];
            }

            //            std::cout << "max = " << JointPosition::MAX_JNT_POSITIONS[i];
            //            std::cout << "; min = " << JointPosition::MIN_JNT_POSITIONS[i] << std::endl;
            //            std::cout << "joint_state = " << joint_state_position[i] << std::endl;
            //            std::cout << "delta pos = " << delta_pos << std::endl;

            delta_pos = std::max(0.0, delta_pos);

            max_velocities[i] = sqrt(2 * delta_pos * ykin::MAX_JNT_ACCELERATIONS[i]);
            max_velocities[i] = std::min(ykin::MAX_JNT_VELOCITIES[i], max_velocities[i]);

            factor = std::min(factor, max_velocities[i] / std::fabs(jnt_velocity[i]));

            if(factor == 0)
            {
                ROS_WARN("MI: Safety factor is 0 in joint %d. Stopping.", i+1);
                break;
            }
        }
    }

    // === SCALE VELOCITIES TO SAFE VALUES ===
    //    if(factor < 1.0)
    //    {
    //        jnt_velocity *= factor;
    //    }

    return factor;
}

//###################### STOP ##########################################################################################
void ModuleInterpolation::stop()
{
    ROS_INFO("MI: Stopping...");

    // === SET FLAGS/STATE ===
    internal_state_ = INACTIVE;
    status_ = STATUS_IDLE;
    arm_is_busy_ = false;
    moving_ = false;

    // === RESET RAMP GENERATORS ===
    ykin::GenericVector zero_velocity;
    ykin::CylindricPosition current_cyl_position = joint_state_position_.toCylindric();
    ykin::GenericVector current_cart_position = current_cyl_position.toCartesian();
    cart_ramp_generator_.resetCurrentState(zero_velocity, current_cart_position);
    cyl_ramp_generator_.resetCurrentState(zero_velocity, current_cyl_position);
    jnt_ramp_generator_.resetCurrentState(zero_velocity, joint_state_position_);
    cart_ramp_generator_.setTargetVelocity(zero_velocity);
    cyl_ramp_generator_.setTargetVelocity(zero_velocity);
    jnt_ramp_generator_.setTargetVelocity(zero_velocity);

    // === RESET TORQUE CONTROLLER ===
    if(command_mode_ == TORQUE)
        torque_controller_->reset();

    // === RESET LAST VELOCITY ===
    last_velocity_ = ykin::JointVelocity();

    // === PUBLISH CURRENT JOINT STATE AS POSITION COMMAND ===
    youbot_->arm()->setJointPositions(joint_state_position_);
}
