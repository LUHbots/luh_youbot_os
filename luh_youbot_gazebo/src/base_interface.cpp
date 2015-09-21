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

#include "luh_youbot_gazebo/base_interface.h"

//########## CONSTRUCTOR ###############################################################################################
YoubotBaseGazeboInterface::YoubotBaseGazeboInterface(string name, YoubotConfiguration &config):
    YoubotBaseInterface(name, config),
    is_initialised_(false)
{
}

//########## DESTRUCTOR ################################################################################################
YoubotBaseGazeboInterface::~YoubotBaseGazeboInterface()
{
}

//########## INITIALISE ################################################################################################
void YoubotBaseGazeboInterface::initialise()
{
    config_->node_handle->param("youbot_oodl_driver/maxLinearVel", this->max_linear_vel_, 0.7);
    config_->node_handle->param("youbot_oodl_driver/maxAngularVel", this->max_angular_vel_, 0.7);

    // === PUBLISHERS ===
    odometry_publisher_ = config_->node_handle->advertise<nav_msgs::Odometry>("odom", 1);
    joint_state_publisher_ = config_->node_handle->advertise<sensor_msgs::JointState>("base/joint_states", 1);

    // === GAZEBO STUFF ===
    cmd_vel_publisher_ = config_->node_handle->advertise<geometry_msgs::Twist>("gazebo/cmd_vel",1);
    odom_subscriber_ = config_->node_handle->subscribe<nav_msgs::Odometry>(
                "odom", 1, &YoubotBaseGazeboInterface::odomCallback, this);

    ROS_INFO("%s is initialised.", name_.c_str());

    is_initialised_ = true;
}

//########## READ STATE ################################################################################################
void YoubotBaseGazeboInterface::readState()
{
    last_sample_time_ = current_sample_time_;
    current_sample_time_ = ros::Time::now();
    delta_t_ = (current_sample_time_ - last_sample_time_).toSec();

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
void YoubotBaseGazeboInterface::updateController()
{
    controller_command_ = velocity_command_;
}

//########## WRITE COMMANDS ############################################################################################
bool YoubotBaseGazeboInterface::writeCommands()
{
    cmd_vel_publisher_.publish(velocity_command_);

    return true;
}

//########## STOP ######################################################################################################
void YoubotBaseGazeboInterface::stop()
{

}

//########## PUBLISH MESSAGES ##########################################################################################
void YoubotBaseGazeboInterface::publishMessages()
{
    joint_state_msg_.header.stamp=current_sample_time_;
    joint_state_publisher_.publish(joint_state_msg_);
}

//########## ODOM CALLBACK #############################################################################################
void YoubotBaseGazeboInterface::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    odom_msg_ = *odom;
}
