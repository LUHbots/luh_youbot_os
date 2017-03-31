/* *****************************************************************
 *
 * luh_youbot_driver_api
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
 *
 *
 *
 * === Based on youbot_oodl with original license header: ===
 *
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Sebastian Blumenthal
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "luh_youbot_gazebo/arm_interface.h"
#include <luh_youbot_msgs/JointVector.h>

namespace ykin = luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
YoubotArmGazeboInterface::YoubotArmGazeboInterface(std::string name, YoubotConfiguration &config):
    YoubotArmInterface(name, config),
    is_initialised_(false)
{

}

//########## DESTRUCTOR ################################################################################################
YoubotArmGazeboInterface::~YoubotArmGazeboInterface()
{
}

//########## INITIALISE ################################################################################################
void YoubotArmGazeboInterface::initialise(bool use_standard_gripper, bool use_luh_gripper_v3)
{
    ROS_INFO("Initialising %s...", name_.c_str());

    // === INITIALISATION ===
    arm_index_ = config_->num_arms++;
    std::stringstream ss;
    ss << "arm_" << arm_index_+1 << "/";
    topic_prefix_ = ss.str();

    gripper_finger_names_.resize(2);
    torque_command_.assign(5, 0.0);
    velocity_command_.assign(5, 0.0);
    position_command_.assign(5, 0.0);
    gripper_command_.assign(2, 0.0);

    joint_names_.resize(5);
    joint_names_[0] = "arm_joint_1";
    joint_names_[1] = "arm_joint_2";
    joint_names_[2] = "arm_joint_3";
    joint_names_[3] = "arm_joint_4";
    joint_names_[4] = "arm_joint_5";

    gripper_finger_names_[LEFT_FINGER_INDEX] = "gripper_finger_joint_l";
    gripper_finger_names_[RIGHT_FINGER_INDEX] = "gripper_finger_joint_r";

    // === ROS COMMUNICATION ===
    std::string topic_name = topic_prefix_;
    topic_name.append("joint_states");
    joint_state_publisher_ = config_->node_handle->advertise<sensor_msgs::JointState>(topic_name, 1);

    // === GAZEBO STUFF ===
    position_command_publisher_ = config_->node_handle->advertise<luh_youbot_msgs::JointVector>(
                "gazebo/joint_position_command", 1);
    velocity_command_publisher_ = config_->node_handle->advertise<luh_youbot_msgs::JointVector>(
                "gazebo/joint_velocity_command", 1);
    torque_command_publisher_ = config_->node_handle->advertise<luh_youbot_msgs::JointVector>(
                "gazebo/joint_torque_command", 1);
    joint_state_subscriber_ = config_->node_handle->subscribe("gazebo/joint_states", 1,
                                                              &YoubotArmGazeboInterface::jointStateCallback, this);

    // === INIT JOINT STATE MESSAGE ===
    for(uint i=0; i<joint_names_.size(); i++)
    {
        joint_state_.name.push_back(joint_names_[i]);
    }
    joint_state_.name.push_back(gripper_finger_names_[LEFT_FINGER_INDEX]);
    joint_state_.name.push_back(gripper_finger_names_[RIGHT_FINGER_INDEX]);

    joint_state_.position.resize(7);
    joint_state_.velocity.resize(7);
    joint_state_.effort.resize(7);

    ROS_INFO("Arm \"%s\" is initialized.", name_.c_str());
    ROS_INFO("System has %i initialized arm(s).", config_->num_arms);

    is_initialised_ = true;
}

//########## SECURITY CHECK ############################################################################################
int YoubotArmGazeboInterface::securityCheck()
{
    return 0;
}

//########## READ STATE ################################################################################################
void YoubotArmGazeboInterface::readState()
{
    last_sample_time_ = current_sample_time_;
    current_sample_time_ = ros::Time::now();
    delta_t_ = (current_sample_time_ - last_sample_time_).toSec();

    joint_position_.setValues(joint_state_.position);
    joint_position_.addOffset();
    joint_velocity_.setValues(joint_state_.velocity);
    joint_torque_.setValues(joint_state_.effort);
}

//########## WRITE COMMANDS ############################################################################################
bool YoubotArmGazeboInterface::writeCommands()
{
    if(has_new_arm_command_)
    {
        luh_youbot_msgs::JointVector msg;

        if(mode_ == POSITION)
        {
            msg.q1 = position_command_[0];
            msg.q2 = position_command_[1];
            msg.q3 = position_command_[2];
            msg.q4 = position_command_[3];
            msg.q5 = position_command_[4];
            position_command_publisher_.publish(msg);
        }
        else if(mode_ == VELOCITY)
        {
            msg.q1 = velocity_command_[0];
            msg.q2 = velocity_command_[1];
            msg.q3 = velocity_command_[2];
            msg.q4 = velocity_command_[3];
            msg.q5 = velocity_command_[4];
            velocity_command_publisher_.publish(msg);
        }
        else
        {
            msg.q1 = torque_command_[0];
            msg.q2 = torque_command_[1];
            msg.q3 = torque_command_[2];
            msg.q4 = torque_command_[3];
            msg.q5 = torque_command_[4];
            torque_command_publisher_.publish(msg);
        }

        has_new_arm_command_ = false;
    }

    // === SET GRIPPER COMMAND ===
    if(has_new_gripper_command_)
    {
        //TODO gripper command

        has_new_gripper_command_ = false;
    }

    return true;
}

//########## STOP ######################################################################################################
void YoubotArmGazeboInterface::stop()
{

}

//########## PUBLISH MESSAGES ##########################################################################################
void YoubotArmGazeboInterface::publishMessages()
{
    joint_state_.header.stamp=current_sample_time_;
    joint_state_publisher_.publish(joint_state_);
}

//########## CALLBACK: JOINT STATE #####################################################################################
void YoubotArmGazeboInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state)
{
    for(uint i=0; i<joint_state->name.size(); i++)
    {
        std::string name = joint_state->name[i];

        int j;
        if(name.compare("arm_joint_1") == 0) j = 0;
        else if(name.compare("arm_joint_2") == 0) j = 1;
        else if(name.compare("arm_joint_3") == 0) j = 2;
        else if(name.compare("arm_joint_4") == 0) j = 3;
        else if(name.compare("arm_joint_5") == 0) j = 4;
        else continue;

        joint_state_.position[j] = joint_state->position[i];
        joint_state_.velocity[j] = joint_state->velocity[i];
        joint_state_.effort[j] = joint_state->effort[i];
    }
}
