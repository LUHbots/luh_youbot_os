/* *****************************************************************
 *
 * luh_youbot_interface
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

#include "luh_youbot_vrep_interface/base_interface.h"

//########## CONSTRUCTOR ###############################################################################################
YoubotBaseVrepInterface::YoubotBaseVrepInterface(string name, YoubotConfiguration &config):
    YoubotBaseInterface(name, config),
    is_initialised_(false)
{
}

//########## DESTRUCTOR ################################################################################################
YoubotBaseVrepInterface::~YoubotBaseVrepInterface()
{
}

//########## INITIALISE ################################################################################################
void YoubotBaseVrepInterface::initialise()
{
    config_->node_handle->param("youbot_oodl_driver/maxLinearVel", this->max_linear_vel_, 0.7);
    config_->node_handle->param("youbot_oodl_driver/maxAngularVel", this->max_angular_vel_, 0.7);

    // === PUBLISHERS ===
    odometry_publisher_ = config_->node_handle->advertise<nav_msgs::Odometry>("odom", 1);
    joint_state_publisher_ = config_->node_handle->advertise<sensor_msgs::JointState>("base/joint_states", 1);

    // === VREP STUFF ===
    pub_cmdvel_ = config_->node_handle->advertise<geometry_msgs::Twist>("vrep/cmd_vel",1);
    sub_odom_twist_ = config_->node_handle->subscribe("/vrep/odom_twist",1,&YoubotBaseVrepInterface::odomTwistCallback,this);
    sub_odom_pose_ = config_->node_handle->subscribe("/vrep/odom_pose",1,&YoubotBaseVrepInterface::odomPoseCallback,this);
    sub_base_joint_states_.push_back(config_->node_handle->subscribe("/vrep/base/joint_states_1",1,&YoubotBaseVrepInterface::jointStateCallback,this));
    sub_base_joint_states_.push_back(config_->node_handle->subscribe("/vrep/base/joint_states_2",1,&YoubotBaseVrepInterface::jointStateCallback,this));
    sub_base_joint_states_.push_back(config_->node_handle->subscribe("/vrep/base/joint_states_3",1,&YoubotBaseVrepInterface::jointStateCallback,this));
    sub_base_joint_states_.push_back(config_->node_handle->subscribe("/vrep/base/joint_states_4",1,&YoubotBaseVrepInterface::jointStateCallback,this));

    // === INITIALISATION ===

    // init odometry msg
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.header.frame_id = "odom";

    // init odometry transform
    odom_tf_.header.frame_id = odom_msg_.header.frame_id;
    odom_tf_.child_frame_id = odom_msg_.child_frame_id;

    // init jointstate msg
    std::vector<std::string> wheel_names;
    wheel_names.push_back("wheel_joint_fl"); //wheel #1
    wheel_names.push_back("wheel_joint_fr"); //wheel #2
    wheel_names.push_back("wheel_joint_bl"); //wheel #3
    wheel_names.push_back("wheel_joint_br"); //wheel #4
    wheel_names.push_back("caster_joint_fl"); // virtual joint #1
    wheel_names.push_back("caster_joint_fr"); // virtual joint #2
    wheel_names.push_back("caster_joint_bl"); // virtual joint #3
    wheel_names.push_back("caster_joint_br"); // virtual joint #4

    for(int i=0; i<8; i++)
    {
        joint_state_msg_.name.push_back(wheel_names[i]);
        joint_state_msg_.position.push_back(0.0);
        joint_state_msg_.velocity.push_back(0.0);
        joint_state_msg_.effort.push_back(0.0);
    }

    joint_states_.resize(4);

    ROS_INFO("%s is initialised.", name_.c_str());

    is_initialised_ = true;
}

//########## READ STATE ################################################################################################
void YoubotBaseVrepInterface::readState()
{
    last_sample_time_ = current_sample_time_;
    current_sample_time_ = ros::Time::now();
    delta_t_ = (current_sample_time_ - last_sample_time_).toSec();

    odom_tf_.header.stamp = current_sample_time_;

    odom_tf_.transform.translation.x = odom_msg_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_msg_.pose.pose.position.y;
    odom_tf_.transform.translation.z = 0.0;
    odom_tf_.transform.rotation = odom_msg_.pose.pose.orientation;

    odom_msg_.header.stamp = current_sample_time_;

    // save position as Pose2D
    current_pose_.x = odom_msg_.pose.pose.position.x;
    current_pose_.y = odom_msg_.pose.pose.position.y;

    current_pose_.theta = tf::getYaw(odom_msg_.pose.pose.orientation);

    if(current_pose_.theta > M_PI)
        current_pose_.theta -= 2*M_PI;
    if(current_pose_.theta < -M_PI)
        current_pose_.theta += 2*M_PI;

}

//########## UPDATE CONTROLLER #########################################################################################
void YoubotBaseVrepInterface::updateController()
{
    controller_command_ = velocity_command_;
}

//########## WRITE COMMANDS ############################################################################################
bool YoubotBaseVrepInterface::writeCommands()
{
    pub_cmdvel_.publish(velocity_command_);

    return true;
}

//########## STOP ######################################################################################################
void YoubotBaseVrepInterface::stop()
{

}

//########## PUBLISH MESSAGES ##########################################################################################
void YoubotBaseVrepInterface::publishMessages()
{
    joint_state_msg_.header.stamp=current_sample_time_;
    odom_broadcaster_.sendTransform(odom_tf_);
    odometry_publisher_.publish(odom_msg_);
    joint_state_publisher_.publish(joint_state_msg_);
}

//########## CALLBACK: VREP ODOM POSE ##################################################################################
void YoubotBaseVrepInterface::odomPoseCallback(const geometry_msgs::PoseStamped& pose)
{
    odom_msg_.pose.pose = pose.pose;
}

//########## CALLBACK: VREP ODOM TWIST #################################################################################
void YoubotBaseVrepInterface::odomTwistCallback(const geometry_msgs::TwistStamped& twist)
{
    odom_msg_.twist.twist = twist.twist;
}

//########## CALLBACK: VREP JOINT STATE ################################################################################
void YoubotBaseVrepInterface::jointStateCallback(const sensor_msgs::JointState& joint_state)
{
    int i = 0;
    int sign = 1;
    if(joint_state.name[0].compare("rollingJoint_fl") == 0) i=0;
    if(joint_state.name[0].compare("rollingJoint_rl") == 0) {i=1; sign = -1;}
    if(joint_state.name[0].compare("rollingJoint_rr") == 0) i=2;
    if(joint_state.name[0].compare("rollingJoint_fr") == 0) {i=3; sign = -1;}

    joint_state_msg_.position[i] = sign * joint_state.position[0];
    joint_state_msg_.velocity[i] = sign * joint_state.velocity[0];
    joint_state_msg_.effort[i] = sign * joint_state.effort[0];
}
