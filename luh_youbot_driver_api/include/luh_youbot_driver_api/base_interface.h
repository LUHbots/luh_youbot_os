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

#ifndef LUH_YOUBOT_BASE_INTERFACE_H
#define LUH_YOUBOT_BASE_INTERFACE_H

#include "luh_youbot_driver_api/common.h"
#include "youbot/YouBotBase.hpp"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

class YoubotBaseInterface
{
public:
    YoubotBaseInterface(std::string name, YoubotConfiguration &config);
    ~YoubotBaseInterface();
    virtual void initialise();
    virtual void readState();
    virtual void updateController();
    virtual bool writeCommands();
    virtual void stop();
    virtual void publishMessages();

    void setVelocity(double v_x, double v_y, double v_theta);
    void setVelocity(const geometry_msgs::Twist &velocity);
    geometry_msgs::Pose2D getPose();

    virtual bool isInitialised(){return base_ != NULL;}

protected:

    struct BaseState
    {
        double position, velocity, current, torque;
    };

    std::vector<BaseState> joint_states_;

    ros::Publisher joint_state_publisher_;
    ros::Publisher odometry_publisher_;

    ros::ServiceServer switch_off_server_;
    ros::ServiceServer switch_on_server_;

    tf::TransformBroadcaster odom_broadcaster_;

    std::string name_;
    YoubotConfiguration *config_;

    youbot::YouBotBase *base_;

    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped odom_tf_;
    geometry_msgs::Twist velocity_command_;
    geometry_msgs::Twist controller_command_;
    geometry_msgs::Twist last_controller_command_;

    sensor_msgs::JointState joint_state_msg_;
    geometry_msgs::Pose2D current_pose_;

    ros::Time current_sample_time_;
    ros::Time last_sample_time_;
    double delta_t_;

    double max_linear_vel_;
    double max_angular_vel_;

    bool motors_are_off_;

    struct ControllerParameters
    {
        double error_alt[3];
        double ctrl_sum[3];		///< sum of I value of the PI-controller
        double moving_average;

        double C_P[3];
        double C_I[3];
        double C_D[3];
        double C_N[3];

        double iSatUpper[3];
        double iSatLower[3];
        double dSatUpper[3];
        double dSatLower[3];

        double C_IM[3];
        double C_DM[3];

        bool is_enabled;
    }controller_;

    bool switchOffBaseMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool switchOnBaseMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

};
#endif // LUH_YOUBOT_BASE_INTERFACE_H
