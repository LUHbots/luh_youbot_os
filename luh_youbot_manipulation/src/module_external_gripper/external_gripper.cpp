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

#include "luh_youbot_manipulation/module_external_gripper/external_gripper.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "ros/package.h"
#include <boost/units/io.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/units/systems/si.hpp>
#include <std_msgs/Float32.h>

//########## CONSTRUCTOR ###############################################################################################
ModuleExternalGripper::ModuleExternalGripper(): ManipulationModule(),
    grip_object_server_(NULL),
    set_gripper_server_(NULL)
{
}

//########## DESTRUCTOR ###############################################################################################
ModuleExternalGripper::~ModuleExternalGripper()
{
    delete grip_object_server_;
    delete set_gripper_server_;
}

//########## INITIALIZATION ############################################################################################
void ModuleExternalGripper::init()
{
    ROS_INFO("Initialising Gripper Module...");
    status_ = STATUS_IDLE;    

    // === PARAMETERS ===
    activated_ = true;
    control_mode_ == NONE;

    node_->param("module_external_gripper/min_gripper_width", min_gripper_width_, 0.0);
    node_->param("module_external_gripper/max_gripper_width", max_gripper_width_, 0.06);
    node_->param("module_external_gripper/grip_force", grip_force_, 6.0);
    node_->param("module_external_gripper/position_tolerance", position_tolerance_, 0.001);
    node_->param("module_external_gripper/force_tolerance", force_tolerance_, 1.0);

    // === ACTION SERVERS ===
    grip_object_server_ = new GripObjectServer(*node_, "arm_1/grip_object", false);
    grip_object_server_->registerGoalCallback(boost::bind(&ModuleExternalGripper::gripObjectCallback, this));
    grip_object_server_->start();
    set_gripper_server_ = new SetGripperServer(*node_, "arm_1/set_gripper", false);
    set_gripper_server_->registerGoalCallback(boost::bind(&ModuleExternalGripper::setGripperCallback, this));
    set_gripper_server_->start();

    // === SUBSCRIBER ===
    gripper_pos_cmd_subscriber_ = node_->subscribe("arm_1/gripper_command", 1, &ModuleExternalGripper::gripperMsgCallback, this);
    gripper_state_subscriber_ = node_->subscribe("gripper/joint_states", 1, &ModuleExternalGripper::jointstateCallback, this);

    // === PUBLISHER ===
    pos_cmd_publisher_ = node_->advertise<std_msgs::Float32>("gripper/position_command", 1);
    frc_cmd_publisher_ = node_->advertise<std_msgs::Float32>("gripper/force_command", 1);

    ROS_INFO("Gripper Module initialised.");
}

//########## UPDATE ####################################################################################################
void ModuleExternalGripper::update()
{
    if(!activated_ || control_mode_ == NONE)
        return;

    if(control_mode_ == POSITION)
    {
        if(fabs(current_position_ - pos_cmd_.data) < position_tolerance_)
        {
            if(opening_gripper_)
                grip_object_server_->setSucceeded();
            else
                set_gripper_server_->setSucceeded();
            control_mode_ = NONE;
            gripper_is_busy_ = false;
            ROS_INFO("Gripper position reached.");
        }
    }
    else if(control_mode_ == FORCE)
    {
        if(fabs(current_force_ - frc_cmd_.data) < force_tolerance_)
        {
            grip_object_server_->setSucceeded();
            control_mode_ = NONE;
            gripper_is_busy_ = false;
            ROS_INFO("Gripper force applied.");
        }
    }
}

//########## ACTIVATE ##################################################################################################
void ModuleExternalGripper::activate()
{
    activated_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleExternalGripper::deactivate()
{
    activated_ = false;
}

//########## EMERGENCY STOP ############################################################################################
void ModuleExternalGripper::emergencyStop()
{
    if(grip_object_server_->isActive())
    {
        grip_object_server_->setAborted();
    }
    if(set_gripper_server_->isActive())
    {
        set_gripper_server_->setAborted();
    }

    gripper_is_busy_ = false;
}

//########## CALLBACK: GRIP OBJECT #####################################################################################
void ModuleExternalGripper::gripObjectCallback()
{
    boost::mutex::scoped_lock lock(gripper_mutex_);

    ROS_INFO("==== Module Gripper ====");

    std::string name = grip_object_server_->acceptNewGoal()->object_name;

    if(name.compare("OPEN") == 0)
    {
        control_mode_ = POSITION;
        pos_cmd_.data = max_gripper_width_;
        pos_cmd_publisher_.publish(pos_cmd_);
        opening_gripper_ = true;
    }
    else
    {
        control_mode_ = FORCE;
        frc_cmd_.data = grip_force_;
        frc_cmd_publisher_.publish(frc_cmd_);
        opening_gripper_ = false;
    }

    gripper_is_busy_ = true;

    ROS_INFO("Gripper command received.");
}

//########## CALLBACK: SET GRIPPER (ACTION) ############################################################################
void ModuleExternalGripper::setGripperCallback()
{
    boost::mutex::scoped_lock lock(gripper_mutex_);

    ROS_INFO("==== Module Gripper ====");

    double new_goal_width = set_gripper_server_->acceptNewGoal()->gripper_width;

    if(new_goal_width > max_gripper_width_ || new_goal_width < min_gripper_width_)
    {
        ROS_ERROR("Requested gripper width of %f is out of range.", new_goal_width);

        set_gripper_server_->setAborted();

        return;
    }

    gripper_is_busy_ = true;
    control_mode_ = POSITION;
    pos_cmd_.data = new_goal_width;
    pos_cmd_publisher_.publish(pos_cmd_);

    ROS_INFO("Gripper command received.");
}

//########## CALLBACK: SET GRIPPER (SUBSCRIBER) ########################################################################
void ModuleExternalGripper::gripperMsgCallback(const std_msgs::Float32::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(gripper_mutex_);

    if(msg->data > max_gripper_width_ || msg->data < min_gripper_width_)
    {
        ROS_ERROR("Requested gripper width of %f is out of range.", msg->data);
        return;
    }

    pos_cmd_.data = msg->data;
    pos_cmd_publisher_.publish(pos_cmd_);
}

//########## CALLBACK: JOINT STATE #####################################################################################
void ModuleExternalGripper::jointstateCallback(const sensor_msgs::JointState &state)
{
    ROS_ASSERT(state.position.size() >= 2 && state.effort.size() >= 2);

    current_position_ = state.position[0] + state.position[1];
    current_force_ = (state.effort[0] + state.effort[1]) / 2; // TODO: mean, max, min?
}
