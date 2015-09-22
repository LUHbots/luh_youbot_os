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

#include "luh_youbot_controller/module_direct_control/module_direct_control.h"

using namespace luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
ModuleDirectControl::ModuleDirectControl():ControllerModule(),
    cartesian_pose_server_(NULL),
    cylindric_pose_server_(NULL),
    joint_pose_server_(NULL),
    named_pose_server_(NULL)
{
}

//########## DESTRUCTOR ################################################################################################
ModuleDirectControl::~ModuleDirectControl()
{
    delete cartesian_pose_server_;
    delete cylindric_pose_server_;
    delete joint_pose_server_;
    delete named_pose_server_;
}

//########## INIT ######################################################################################################
void ModuleDirectControl::init()
{
    ROS_INFO("MDC: Initialising Direct Control Module...");

    // === INIT ===
    active_ = true;
    has_command_ = false;

    // === PARAMETERS ===
    node_->param("module_direct_control/joint_position_tolerance", joint_position_tolerance_, 1.0);
    joint_position_tolerance_ *= (M_PI/180.0);
    node_->param("module_direct_control/joint_velocity_tolerance", joint_velocity_tolerance_, 0.1);
    joint_velocity_tolerance_ *= (M_PI/180.0);


    // === ACTION SERVERS ===
    this->cartesian_pose_server_ = new CartesianServer(*this->node_, "arm_1/to_cartesian_pose/direct", false);
    this->cartesian_pose_server_->registerGoalCallback(
                boost::bind(&ModuleDirectControl::cartesianPoseCallback, this));
    this->cylindric_pose_server_ = new CylindricServer(*this->node_, "arm_1/to_cylindric_pose/direct", false);
    this->cylindric_pose_server_->registerGoalCallback(
                boost::bind(&ModuleDirectControl::cylindricPoseCallback, this));
    this->joint_pose_server_ = new JointServer(*this->node_, "arm_1/to_joint_pose/direct", false);
    this->joint_pose_server_->registerGoalCallback(boost::bind(&ModuleDirectControl::jointPoseCallback, this));

    this->named_pose_server_ = new NamedServer(*this->node_, "arm_1/to_named_pose/direct", false);
    this->named_pose_server_->registerGoalCallback(boost::bind(&ModuleDirectControl::namedPoseCallback, this));

    cartesian_pose_server_->registerPreemptCallback(boost::bind(&ModuleDirectControl::preemptCallback, this));
    cylindric_pose_server_->registerPreemptCallback(boost::bind(&ModuleDirectControl::preemptCallback, this));
    joint_pose_server_->registerPreemptCallback(boost::bind(&ModuleDirectControl::preemptCallback, this));
    named_pose_server_->registerPreemptCallback(boost::bind(&ModuleDirectControl::preemptCallback, this));

    cartesian_pose_server_->start();
    cylindric_pose_server_->start();
    joint_pose_server_->start();
    named_pose_server_->start();

    // === SUBSCRIBER ===
    joint_position_subscriber_ = node_->subscribe("arm_1/joint_position_command", 10,
                                                  &ModuleDirectControl::jointPositionCallback, this);

    // === SERVICE SERVERS ===
    relax_server_ = node_->advertiseService("arm_1/relax", &ModuleDirectControl::relaxCallback, this);
    stiffen_server_ = node_->advertiseService("arm_1/stiffen", &ModuleDirectControl::stiffenCallback, this);

    ROS_INFO("MDC: Direct Control Module initialised.");
}

//########## UPDATE ####################################################################################################
void ModuleDirectControl::update()
{
    if(!(has_command_ && active_))
        return;

    // === CHECK DISTANCE FROM GOAL POSITION ===

    JointPosition pos_diff = position_command_ - youbot_->arm()->getJointPosition();
    JointVelocity velocity = youbot_->arm()->getJointVelocity();

    for(uint i=0; i<pos_diff.size(); i++)
    {
        if(fabs(pos_diff[i]) > joint_position_tolerance_
                || fabs(velocity[i] > joint_velocity_tolerance_))
        {
            return;
        }
    }

    // === IF GOAL IS REACHED: SET SUCCEEDED ===

    if(cartesian_pose_server_->isActive())
        cartesian_pose_server_->setSucceeded();

    if(cylindric_pose_server_->isActive())
        cylindric_pose_server_->setSucceeded();

    if(joint_pose_server_->isActive())
        joint_pose_server_->setSucceeded();

    if(named_pose_server_->isActive())
        named_pose_server_->setSucceeded();

    has_command_ = false;
    arm_is_busy_ = false;

    ROS_INFO("Goal position reached.");
}

//########## ACTIVATE ##################################################################################################
void ModuleDirectControl::activate()
{
    active_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleDirectControl::deactivate()
{
    preempt();
    active_ = false;
}

//###################### EMERGENCY STOP ################################################################################
void ModuleDirectControl::emergencyStop()
{
    bool aborted = false;

    if(cartesian_pose_server_->isActive())
    {
        aborted = true;
        cartesian_pose_server_->setAborted();
    }
    if(cylindric_pose_server_->isActive())
    {
        aborted = true;
        cylindric_pose_server_->setAborted();
    }
    if(joint_pose_server_->isActive())
    {
        aborted = true;
        joint_pose_server_->setAborted();
    }
    if(named_pose_server_->isActive())
    {
        aborted = true;
        joint_pose_server_->setAborted();
    }
}

//###################### CALLBACK: PREEMPT #############################################################################
void ModuleDirectControl::preemptCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);
    preempt();

}

//###################### PREEMPT #######################################################################################
void ModuleDirectControl::preempt()
{
    bool preempted = false;

    if(cartesian_pose_server_->isActive())
    {
        preempted = true;
        cartesian_pose_server_->setPreempted();
    }
    if(cylindric_pose_server_->isActive())
    {
        preempted = true;
        cylindric_pose_server_->setPreempted();
    }
    if(joint_pose_server_->isActive())
    {
        preempted = true;
        joint_pose_server_->setPreempted();
    }
    if(named_pose_server_->isActive())
    {
        preempted = true;
        joint_pose_server_->setPreempted();
    }

    if(preempted)
    {
        luh_youbot_kinematics::JointPosition current_pos = youbot_->arm()->getJointPosition();
        youbot_->arm()->setJointPositions(current_pos);
        ROS_INFO("Action preempted.");
    }
}

//###################### CALLBACK: CARTESIAN POSITION ##################################################################
void ModuleDirectControl::cartesianPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ROS_INFO("==== Module Direct Control ====");

    if(!active_ || arm_is_busy_)
    {
        ROS_WARN("MDC: Can't accept action goals.");
        cartesian_pose_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::MoveToCartesianPoseGoal> goal = cartesian_pose_server_->acceptNewGoal();

    CartesianPosition position;

    try
    {
        position = CartesianPosition::fromMsg(goal->pose, goal->pose_is_relative,
                                              youbot_->arm()->getJointPosition(), tf_listener_);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("MDC: %s",ex.what());
        cartesian_pose_server_->setAborted();
        return;
    }

    position.printValues("Received position command.");

    // === GET JOINTSPACE END POINT ===
    position_command_ = position.toJointspace();

    // === CHECK REACHABILITY ===
    if(!(position.isReachable() && position_command_.isReachable()))
    {
        if(!position.isReachable())
            ROS_ERROR("MTPT: Cartesian position is unreachable.");
        else
            ROS_ERROR("MDC: Pose exceeds joint limits.");
        cartesian_pose_server_->setAborted();
        return;
    }

    arm_is_busy_ = true;
    has_command_ = true;

    youbot_->arm()->setJointPositions(position_command_);

    ROS_INFO("Goal position set.");
}

//###################### CALLBACK: CYLINDRIC POSITION ##################################################################
void ModuleDirectControl::cylindricPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ROS_INFO("==== Module Direct Control ====");

    if(!active_ || arm_is_busy_)
    {
        ROS_WARN("MDC: Can't accept action goals.");
        cylindric_pose_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::MoveToCylindricPoseGoal> goal = cylindric_pose_server_->acceptNewGoal();

    CylindricPosition cyl_pos;
    cyl_pos.setQ1(goal->pose.q1);
    cyl_pos.setR(goal->pose.r);
    cyl_pos.setZ(goal->pose.z);
    cyl_pos.setTheta(goal->pose.theta);
    cyl_pos.setQ5(goal->pose.q5);

    if(goal->pose_is_relative)
        cyl_pos += youbot_->arm()->getJointPosition().toCylindric();

    // === GET JOINTSPACE END POINT ===
    position_command_ = cyl_pos.toJointspace();

    // === CHECK REACHABILITY ===
    if(!(cyl_pos.isReachable() && position_command_.isReachable()))
    {
        ROS_ERROR("MDC: Position is unreachable.");
        cylindric_pose_server_->setAborted();
        return;
    }

    arm_is_busy_ = true;
    has_command_ = true;

    youbot_->arm()->setJointPositions(position_command_);

    ROS_INFO("Goal position set.");
}

//###################### CALLBACK: JOINT POSITION ######################################################################
void ModuleDirectControl::jointPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ROS_INFO("==== Module Direct Control ====");

    if(!active_ || arm_is_busy_)
    {
        ROS_WARN("MDC: Can't accept action goals.");
        joint_pose_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::MoveToJointPoseGoal> goal = joint_pose_server_->acceptNewGoal();

    position_command_.setQ1(goal->pose.q1);
    position_command_.setQ2(goal->pose.q2);
    position_command_.setQ3(goal->pose.q3);
    position_command_.setQ4(goal->pose.q4);
    position_command_.setQ5(goal->pose.q5);

    if(goal->pose_is_relative)
        position_command_ += youbot_->arm()->getJointPosition();

    if(!position_command_.isReachable())
    {
        ROS_ERROR("MDC: Position is unreachable.");
        joint_pose_server_->setAborted();
        return;
    }

    arm_is_busy_ = true;
    has_command_ = true;

    youbot_->arm()->setJointPositions(position_command_);

    ROS_INFO("Goal position set.");
}

//###################### CALLBACK: NAMED POSITION ######################################################################
void ModuleDirectControl::namedPoseCallback()
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    ROS_INFO("==== Module Direct Control ====");

    if(!active_ || arm_is_busy_)
    {
        ROS_WARN("MDC: Can't accept action goals.");
        named_pose_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::MoveToNamedPoseGoal> goal = named_pose_server_->acceptNewGoal();

    if(predefined_poses_.find(goal->pose_name) == predefined_poses_.end())
    {
        ROS_ERROR("Requested Pose '%s is unknown.", goal->pose_name.c_str());
        named_pose_server_->setAborted();
        return;
    }

    position_command_ = predefined_poses_[goal->pose_name];

    if(!position_command_.isReachable())
    {
        ROS_ERROR("MDC: Position is unreachable.");
        named_pose_server_->setAborted();
        return;
    }

    arm_is_busy_ = true;
    has_command_ = true;

    youbot_->arm()->setJointPositions(position_command_);

    ROS_INFO("Goal position set.");
}

//###################### RELAX SERVICE CALLBACK ########################################################################
bool ModuleDirectControl::relaxCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(arm_is_busy_)
        return false;

    ROS_INFO("Arm is relaxed.");
    luh_youbot_kinematics::JointVector torque;
    youbot_->arm()->setJointTorques(torque);
    return true;
}
//###################### STIFFEN SERVICE CALLBACK ######################################################################
bool ModuleDirectControl::stiffenCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(arm_is_busy_)
        return false;

    ROS_INFO("Arm is stiffened.");
    luh_youbot_kinematics::JointPosition pos = youbot_->arm()->getJointPosition();
    youbot_->arm()->setJointPositions(pos);

    return true;
}

//###################### JOINT POSITION CALLBACK #######################################################################
void ModuleDirectControl::jointPositionCallback(const luh_youbot_msgs::JointVector::ConstPtr pos)
{
    boost::mutex::scoped_lock lock(arm_mutex_);

    if(!arm_is_busy_)
    {
        luh_youbot_kinematics::JointPosition jpos;
        jpos.setQ1(pos->q1);
        jpos.setQ2(pos->q2);
        jpos.setQ3(pos->q3);
        jpos.setQ4(pos->q4);
        jpos.setQ5(pos->q5);
        youbot_->arm()->setJointPositions(jpos);
    }
}
