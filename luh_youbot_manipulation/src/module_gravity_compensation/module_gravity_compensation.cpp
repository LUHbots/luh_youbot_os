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

#include "luh_youbot_manipulation/module_gravity_compensation/module_gravity_compensation.h"

using namespace luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
ModuleGravityCompensation::ModuleGravityCompensation():ManipulationModule(),
  is_active_(false),
  do_compensation_(false),
  force_fit_server_(NULL),
  force_(0)
{
}

//########## DESTRUCTOR ################################################################################################
ModuleGravityCompensation::~ModuleGravityCompensation()
{
    delete force_fit_server_;
}

//########## INIT ######################################################################################################
void ModuleGravityCompensation::init()
{
    ROS_INFO("MGC: Initialising Gravity Compensation Module...");

    // === PARAMETERS ===
    int error = 0;

    if(!ros::param::get("module_gravity_compensation/mass_5", params_.mass_5) ) error++;
    if(!ros::param::get("module_gravity_compensation/mass_4", params_.mass_4) ) error++;
    if(!ros::param::get("module_gravity_compensation/mass_3", params_.mass_3) ) error++;
    if(!ros::param::get("module_gravity_compensation/mass_2", params_.mass_2) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_radius_5", params_.com_radius_5) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_radius_4", params_.com_radius_4) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_radius_3", params_.com_radius_3) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_radius_2", params_.com_radius_2) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_angle_5", params_.com_angle_5) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_angle_4", params_.com_angle_4) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_angle_3", params_.com_angle_3) ) error++;
    if(!ros::param::get("module_gravity_compensation/com_angle_2", params_.com_angle_2) ) error++;
    if(!ros::param::get("module_gravity_compensation/friction_5", params_.friction_5) ) error++;
    if(!ros::param::get("module_gravity_compensation/friction_4", params_.friction_4) ) error++;
    if(!ros::param::get("module_gravity_compensation/friction_3", params_.friction_3) ) error++;
    if(!ros::param::get("module_gravity_compensation/friction_2", params_.friction_2) ) error++;
    if(!ros::param::get("module_gravity_compensation/gravity", params_.gravity) ) error++;
    if(!ros::param::get("module_gravity_compensation/buffer_size", buffer_size_) ) error++;

    if(error)
    {
        ROS_ERROR("MGC: %d parameters could not be loaded from parameter server.", error);
    }

    // === ACTION SERVER ===
    force_fit_server_ = new ForceFitServer(*this->node_, "arm_1/force_fit", false);
    force_fit_server_->registerGoalCallback(boost::bind(&ModuleGravityCompensation::forceFitCallback, this));
    force_fit_server_->registerPreemptCallback(boost::bind(&ModuleGravityCompensation::preemptCallback, this));
    force_fit_server_->start();

    // === SERVICE SERVERS ===
    compensate_server_ = node_->advertiseService("arm_1/compensate_gravity",
                                                 &ModuleGravityCompensation::compensateCallback, this);
    deactivate_server_ = node_->advertiseService("arm_1/deactivate_gravity_compensation",
                                                 &ModuleGravityCompensation::deactivateCallback, this);
    get_load_server_ = node_->advertiseService("arm_1/get_load", &ModuleGravityCompensation::getLoadCallback, this);

    is_active_ = true;
    force_ = 0;

    dynamics_.setStaticParameters(params_);

    ROS_INFO("MGC: Gravity Compensation Module initialised.");
}

//########## UPDATE ####################################################################################################
void ModuleGravityCompensation::update()
{
    if(!is_active_)
        return;

    // === BUFFER LAST JOINT STATES ===
    if(position_buffer_.size() == buffer_size_)
    {
        position_buffer_.pop_front();
        torque_buffer_.pop_front();
    }
    position_buffer_.push_back(youbot_->arm()->getJointPosition());
    torque_buffer_.push_back(youbot_->arm()->getJointTorque());

    if(arm_is_busy_)
    {
        do_compensation_ = false;
    }

    // === GRAVITY COMPENSATION ===
    if(do_compensation_)
    {
        JointPosition pos = youbot_->arm()->getJointPosition();

        JointVector effort = dynamics_.getStaticJointEffort(pos);

        bool goal_is_reached = false;
        if(force_ != 0)
        {
            goal_is_reached = checkGoal(pos);
            effort += getEffortFromForce(pos);
        }

        if(goal_is_reached)
            youbot_->arm()->setJointVelocities(JointVelocity());
        else
            youbot_->arm()->setJointTorques(effort);
    }
}

//########## ACTIVATE ##################################################################################################
void ModuleGravityCompensation::activate()
{
    is_active_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleGravityCompensation::deactivate()
{
    is_active_ = false;
}

//########## EMERGENCY STOP ############################################################################################
void ModuleGravityCompensation::emergencyStop()
{
    if(force_fit_server_->isActive())
    {
        force_fit_server_->setAborted();
    }

    force_ = 0;
    arm_is_busy_ = false;
    do_compensation_ = true;
}

//########## COMPENSATE CALLBACK #######################################################################################
bool ModuleGravityCompensation::compensateCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if(!arm_is_busy_)
        do_compensation_ = true;

    return true;
}

//########## DEACTIVATE CALLBACK #######################################################################################
bool ModuleGravityCompensation::deactivateCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    do_compensation_ = false;
    youbot_->arm()->setJointVelocities(JointVelocity());
    return true;
}

//########## DEACTIVATE ################################################################################################
bool ModuleGravityCompensation::getLoadCallback(luh_youbot_msgs::GetLoad::Request &req,
                                                luh_youbot_msgs::GetLoad::Response &res)
{    
    if(position_buffer_.empty())
    {
        ROS_ERROR("MGC: Cannot calculate load, buffer is empty.");
        return false;
    }
    else if(position_buffer_.size() < buffer_size_)
    {
        ROS_WARN("MGC: Only %d elements are in the buffer. Calculation of load might be unprecise.",
                 (int)position_buffer_.size());
    }
    JointPosition pos;
    JointEffort current_effort;

    int n = position_buffer_.size();
    for(int i=0; i<n; i++)
    {
        pos += position_buffer_[i];
        current_effort += torque_buffer_[i];
    }

    pos /= n;
    current_effort /= n;

    JointEffort gravity_effort = dynamics_.getStaticJointEffort(pos);

    JointEffort load_effort = current_effort - gravity_effort;

    CartesianWrench load = load_effort.toCartesian(pos);

    res.load.x = load.x();
    res.load.y = load.y();
    res.load.z = load.z();
    res.load.theta = load.theta();
    res.load.q5 = load.q5();
    res.load.header.frame_id = "arm_link_0";
    res.load.header.stamp = ros::Time::now();

    return true;
}

//########## FORCE FIT CALLBACK ########################################################################################
void ModuleGravityCompensation::forceFitCallback()
{
    if(!is_active_ || arm_is_busy_)
    {
        ROS_WARN("MGC: Can't accept action goals.");
        force_fit_server_->setAborted();
        return;
    }

    boost::shared_ptr<const luh_youbot_msgs::ForceFitGoal> goal = force_fit_server_->acceptNewGoal();

    force_ = goal->force;
    displacement_ = goal->displacement;
    do_compensation_ = true;

    CylindricPosition pos = youbot_->arm()->getJointPosition().toCylindric();
    start_r_ = pos.r();
    start_z_ = pos.z();
}

//########## PREEMPT CALLBACK ##########################################################################################
void ModuleGravityCompensation::preemptCallback()
{
    if(force_fit_server_->isActive())
    {
        force_fit_server_->setPreempted();
        force_ = 0;
        do_compensation_ = false;
        youbot_->arm()->setJointVelocities(JointVelocity());
    }
}

//########## GET EFFORT FROM FORCE #####################################################################################
JointEffort ModuleGravityCompensation::getEffortFromForce(const JointPosition &pos)
{
    double theta = pos.q2() + pos.q3() + pos.q4();

    // transform force from tcp frame
    CylindricWrench wrench;
    wrench.setR(force_ * sin(theta));
    wrench.setZ(force_ * cos(theta));

    return wrench.toJointspace(pos);
}

//########## CHECK GOAL ################################################################################################
bool ModuleGravityCompensation::checkGoal(const luh_youbot_kinematics::JointPosition &pos)
{
    CylindricPosition cyl_pos = pos.toCylindric();
    double dr = cyl_pos.r() - start_r_;
    double dz = cyl_pos.z() - start_z_;

    if(dr*dr+dz*dz > displacement_*displacement_)
    {
        force_ = 0;
        do_compensation_ = false;
        force_fit_server_->setSucceeded();
        return true;
    }
    else
        return false;
}
