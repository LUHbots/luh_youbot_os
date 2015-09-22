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

#include "luh_youbot_vrep_api/arm_interface.h"
#include <std_msgs/Float64.h>

namespace ykin = luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
YoubotArmVrepInterface::YoubotArmVrepInterface(std::string name, YoubotConfiguration &config):
    YoubotArmInterface(name, config),
    is_initialised_(false)
{

}

//########## DESTRUCTOR ################################################################################################
YoubotArmVrepInterface::~YoubotArmVrepInterface()
{
}

//########## INITIALISE ################################################################################################
void YoubotArmVrepInterface::initialise(bool use_standard_gripper)
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

    // === VREP STUFF ===

    for(int i=0; i<5; i++)
    {
        std::stringstream ss1;
        ss1 << "vrep/arm_1/position_command_" << i+1;
        pub_arm_position_command_.push_back(config_->node_handle->advertise<std_msgs::Float64>(ss1.str(), 1));
        std::stringstream ss2;
        ss2 << "vrep/arm_1/joint_states_" << i+1;
        sub_arm_joint_states_.push_back(
                    config_->node_handle->subscribe(ss2.str(), 1,&YoubotArmVrepInterface::armJointStateCallback,this));
    }
    pub_gripper_l_position_command_ =
            config_->node_handle->advertise<std_msgs::Float64>("vrep/gripper_1/position_command_l",1);
    pub_gripper_r_position_command_ =
            config_->node_handle->advertise<std_msgs::Float64>("vrep/gripper_1/position_command_r",1);

    sub_gripper_l_joint_states_ = config_->node_handle->subscribe(
                "vrep/arm_1/gripper/joint_states_l",1,&YoubotArmVrepInterface::gripperJointStateCallback, this);
    sub_gripper_r_joint_states_ = config_->node_handle->subscribe(
                "vrep/arm_1/gripper/joint_states_r",1,&YoubotArmVrepInterface::gripperJointStateCallback, this);

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
bool YoubotArmVrepInterface::securityCheck()
{
    return true;
}

//########## READ STATE ################################################################################################
void YoubotArmVrepInterface::readState()
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
bool YoubotArmVrepInterface::writeCommands()
{
    if(has_new_arm_command_)
    {
        std_msgs::Float64 msg;

        if(mode_ == POSITION)
        {
            for(uint i=0;i<pub_arm_position_command_.size();i++)
            {
                msg.data = position_command_[i];
                this->pub_arm_position_command_[i].publish(msg);
            }
        }
        else if(mode_ == VELOCITY)
        {
//            for(uint i=0;i<pub_arm_velocity_command_.size();i++)
//            {
//                msg.data = velocity_command_[i];
//                this->pub_arm_velocity_command_[i].publish(msg);
//            }
        }
        else
        {
//            for(uint i=0;i<pub_arm_torque_command_.size();i++)
//            {
//                msg.data = torque_command_[i];
//                this->pub_arm_torque_command_[i].publish(msg);
//            }
        }

        has_new_arm_command_ = false;
    }

    // === SET GRIPPER COMMAND ===
    if(has_new_gripper_command_)
    {
        std_msgs::Float64 msg;
        msg.data = gripper_command_[LEFT_FINGER_INDEX];
        pub_gripper_l_position_command_.publish(msg);
        msg.data = gripper_command_[RIGHT_FINGER_INDEX];
        pub_gripper_r_position_command_.publish(msg);

        has_new_gripper_command_ = false;
    }


    return true;
}

//########## STOP ######################################################################################################
void YoubotArmVrepInterface::stop()
{

}

//########## PUBLISH MESSAGES ##########################################################################################
void YoubotArmVrepInterface::publishMessages()
{
    joint_state_.header.stamp=current_sample_time_;
    joint_state_publisher_.publish(joint_state_);
}

//########## CALLBACK: VREP ARM JOINT STATE ############################################################################
void YoubotArmVrepInterface::armJointStateCallback(const sensor_msgs::JointState& joint_state)
{
    int i=0;
    std::string name = joint_state.name[0];

    if(name.compare("arm_joint_1") == 0) i = 0;
    else if(name.compare("arm_joint_2") == 0) i = 1;
    else if(name.compare("arm_joint_3") == 0) i = 2;
    else if(name.compare("arm_joint_4") == 0) i = 3;
    else if(name.compare("arm_joint_5") == 0) i = 4;
    else return;

    joint_state_.position[i] = joint_state.position[0];
    joint_state_.velocity[i] = joint_state.velocity[0];
    joint_state_.effort[i] = joint_state.effort[0];

}

//########## CALLBACK: VREP GRIPPER JOINT STATE ########################################################################
void YoubotArmVrepInterface::gripperJointStateCallback(const sensor_msgs::JointState& joint_state)
{
    int i=0;
    std::string name = joint_state.name[0];
    if(name.compare("gripper_finger_joint_l") == 0) i = 5;
    else if(name.compare("gripper_finger_joint_r") == 0) i = 6;
    else return;

    joint_state_.position[i] = joint_state.position[0];
    joint_state_.velocity[i] = joint_state.velocity[0];
    joint_state_.effort[i] =  joint_state.effort[0];
}
