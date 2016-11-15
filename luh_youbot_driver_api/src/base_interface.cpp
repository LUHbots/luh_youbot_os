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

#include "luh_youbot_driver_api/base_interface.h"

//########## CONSTRUCTOR ###############################################################################################
YoubotBaseInterface::YoubotBaseInterface(string name, YoubotConfiguration &config):
    name_(name),
    config_(&config),
    motors_are_off_(false),
    base_(NULL)
{

}

//########## DESTRUCTOR ################################################################################################
YoubotBaseInterface::~YoubotBaseInterface()
{
    delete base_;
}

//########## INITIALISE ################################################################################################
void YoubotBaseInterface::initialise()
{
    // === PARAMETERS ===
    config_->node_handle->param("youbot_oodl_driver/controller_enabled", controller_.is_enabled, false);
    config_->node_handle->param("youbot_oodl_driver/maxLinearVel", this->max_linear_vel_, 0.7);
    config_->node_handle->param("youbot_oodl_driver/maxAngularVel", this->max_angular_vel_, 0.7);
    config_->node_handle->param("youbot_oodl_driver/moving_average", controller_.moving_average, 0.3);

    config_->node_handle->param("youbot_oodl_driver/C_P_x", controller_.C_P[0], 0.1);
    config_->node_handle->param("youbot_oodl_driver/C_P_y", controller_.C_P[1], 0.1);
    config_->node_handle->param("youbot_oodl_driver/C_P_z", controller_.C_P[2], 0.1);
    config_->node_handle->param("youbot_oodl_driver/C_I_x", controller_.C_I[0], 1.7);
    config_->node_handle->param("youbot_oodl_driver/C_I_y", controller_.C_I[1], 1.7);
    config_->node_handle->param("youbot_oodl_driver/C_I_z", controller_.C_I[2], 1.7);
    config_->node_handle->param("youbot_oodl_driver/C_D_x", controller_.C_D[0], 0.0);
    config_->node_handle->param("youbot_oodl_driver/C_D_y", controller_.C_D[1], 0.0);
    config_->node_handle->param("youbot_oodl_driver/C_D_z", controller_.C_D[2], 0.0);
    config_->node_handle->param("youbot_oodl_driver/C_N_x", controller_.C_N[0], 0.0);
    config_->node_handle->param("youbot_oodl_driver/C_N_y", controller_.C_N[1], 0.0);
    config_->node_handle->param("youbot_oodl_driver/C_N_z", controller_.C_N[2], 0.0);

    config_->node_handle->param("youbot_oodl_driver/C_iSatUpper_x", controller_.iSatUpper[0], 0.4);
    config_->node_handle->param("youbot_oodl_driver/C_iSatUpper_y", controller_.iSatUpper[1], 0.4);
    config_->node_handle->param("youbot_oodl_driver/C_iSatUpper_z", controller_.iSatUpper[2], 0.4);

    config_->node_handle->param("youbot_oodl_driver/C_iSatLower_x", controller_.iSatLower[0], -0.4);
    config_->node_handle->param("youbot_oodl_driver/C_iSatLower_y", controller_.iSatLower[1], -0.4);
    config_->node_handle->param("youbot_oodl_driver/C_iSatLower_z", controller_.iSatLower[2], -0.4);

    config_->node_handle->param("youbot_oodl_driver/C_dSatUpper_x", controller_.dSatUpper[0], 0.2);
    config_->node_handle->param("youbot_oodl_driver/C_dSatUpper_y", controller_.dSatUpper[1], 0.2);
    config_->node_handle->param("youbot_oodl_driver/C_dSatUpper_z", controller_.dSatUpper[2], 0.2);

    config_->node_handle->param("youbot_oodl_driver/C_dSatLower_x", controller_.dSatLower[0], -0.2);
    config_->node_handle->param("youbot_oodl_driver/C_dSatLower_y", controller_.dSatLower[1], -0.2);
    config_->node_handle->param("youbot_oodl_driver/C_dSatLower_z", controller_.dSatLower[2], -0.2);

    // === CONTROLLER INITIALISATION ===
    controller_.error_alt[0] = 0.0;
    controller_.error_alt[1] = 0.0;
    controller_.error_alt[2] = 0.0;
    controller_.ctrl_sum[0] = 0.0;
    controller_.ctrl_sum[1] = 0.0;
    controller_.ctrl_sum[2] = 0.0;
    controller_.C_IM[0] = 0.0;
    controller_.C_IM[1] = 0.0;
    controller_.C_IM[2] = 0.0;
    controller_.C_DM[0] = 0.0;
    controller_.C_DM[1] = 0.0;
    controller_.C_DM[2] = 0.0;

    // === INIT BASE ===
    try
    {
        ROS_INFO("Initialising %s...", name_.c_str());
        ROS_INFO("Configuration file path: %s", config_->config_path.c_str());
        base_ = new youbot::YouBotBase(name_, config_->config_path);
        base_->doJointCommutation();
    }
    catch (std::exception& e)
    {
        std::string error_message = e.what();
        ROS_FATAL("Cannot open youBot driver: \n %s ", error_message.c_str());
        ROS_ERROR("Base \"%s\" could not be initialized.", name_.c_str());
        config_->has_base = false;
        delete base_;
        base_ = NULL;
        return;
    }

    // === PUBLISHERS ===

    odometry_publisher_ = config_->node_handle->advertise<nav_msgs::Odometry>("odom", 1);
    joint_state_publisher_ = config_->node_handle->advertise<sensor_msgs::JointState>("base/joint_states", 1);
    //    joint_current_publisher_ = node.advertise<std_msgs::Float64MultiArray>("base/joint_currents", 1);
    //    joint_torque_publisher_ = node.advertise<std_msgs::Float64MultiArray>("base/joint_torques", 1);

    // === SERVICE SERVERS ===
    switch_off_server_ = config_->node_handle->advertiseService(
                "base/switchOffMotors", &YoubotBaseInterface::switchOffBaseMotorsCallback, this);
    switch_on_server_ = config_->node_handle->advertiseService(
                "base/switchOnMotors", &YoubotBaseInterface::switchOnBaseMotorsCallback, this);

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
//    wheel_names.push_back("caster_joint_fl"); // virtual joint #1
//    wheel_names.push_back("caster_joint_fr"); // virtual joint #2
//    wheel_names.push_back("caster_joint_bl"); // virtual joint #3
//    wheel_names.push_back("caster_joint_br"); // virtual joint #4

    for(int i=0; i<4; i++)
    {
        joint_state_msg_.name.push_back(wheel_names[i]);
        joint_state_msg_.position.push_back(0.0);
        joint_state_msg_.velocity.push_back(0.0);
        joint_state_msg_.effort.push_back(0.0);
    }

    joint_states_.resize(4);

    ROS_INFO("%s is initialised.", name_.c_str());
}

//########## READ STATE ################################################################################################
void YoubotBaseInterface::readState()
{
    if(base_ == NULL)
        return;

    last_sample_time_ = current_sample_time_;
    current_sample_time_ = ros::Time::now();
    delta_t_ = (current_sample_time_ - last_sample_time_).toSec();

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;

    quantity<si::length> longitudinalPosition;
    quantity<si::length> transversalPosition;
    quantity<plane_angle> orientation;

    quantity<si::velocity> longitudinalVelocity;
    quantity<si::velocity> transversalVelocity;
    quantity<si::angular_velocity> angularVelocity;

    // ensure that all joint values will be sent at the same time
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);

    youbot::JointSensedAngle current_angle;
    youbot::JointSensedVelocity current_velocity;
    std::vector<youbot::JointSensedCurrent> current_current;
    std::vector<youbot::JointSensedTorque> current_torque;
    current_current.resize(4);
    current_torque.resize(4);

    try
    {
        base_->getBasePosition(longitudinalPosition, transversalPosition, orientation);
        x = longitudinalPosition.value();
        y = transversalPosition.value();
        theta = orientation.value();

        base_->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        vx = longitudinalVelocity.value();
        vy = transversalVelocity.value();
        vtheta = angularVelocity.value();

        odom_tf_.header.stamp = current_sample_time_;

        odom_tf_.transform.translation.x = x;
        odom_tf_.transform.translation.y = y;
        odom_tf_.transform.translation.z = 0.0;
        odom_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

        /* Setup odometry Message */
        odom_msg_.header.stamp = current_sample_time_;

        odom_msg_.pose.pose.position.x = x;
        odom_msg_.pose.pose.position.y = y;
        odom_msg_.pose.pose.position.z = 0.0;
        odom_msg_.pose.pose.orientation = odom_tf_.transform.rotation;

        odom_msg_.twist.twist.linear.x = vx;
        odom_msg_.twist.twist.linear.y = vy;
        odom_msg_.twist.twist.angular.z = vtheta;

        /* Set up joint state message for the wheels */
        joint_state_msg_.header.stamp = current_sample_time_;

        for (int i = 0; i < 4; ++i)
        {
            base_->getBaseJoint(i + 1).getData(current_angle); //youBot joints start with 1 not with 0 -> i + 1
            base_->getBaseJoint(i + 1).getData(current_velocity);

            joint_state_msg_.position[i] = current_angle.angle.value();
            joint_state_msg_.velocity[i] = current_velocity.angularVelocity.value();
        }

        /*
     * Yet another hack to make the published values compatible with the URDF description.
     * We actually flipp the directions of the wheel on the right side such that the standard ROS controllers
     * (e.g. for PR2) can be used for the youBot
     */
        joint_state_msg_.position[0] = -joint_state_msg_.position[0];
        joint_state_msg_.position[2] = -joint_state_msg_.position[2];

        base_->getJointData(current_current);
        base_->getJointData(current_torque);

        for(int i=0; i<4; i++)
        {
            joint_states_[i].position = joint_state_msg_.position[i];
            joint_states_[i].velocity = joint_state_msg_.velocity[i];
            joint_states_[i].current = current_current[i].current.value();
            joint_states_[i].torque = current_torque[i].torque.value();
            joint_state_msg_.effort[i] = joint_states_[i].torque;
        }

        // save position as Pose2D
        current_pose_.x = odom_msg_.pose.pose.position.x;
        current_pose_.y = odom_msg_.pose.pose.position.y;

        current_pose_.theta = tf::getYaw(odom_msg_.pose.pose.orientation);

        if(current_pose_.theta > M_PI)
            current_pose_.theta -= 2*M_PI;
        if(current_pose_.theta < -M_PI)
            current_pose_.theta += 2*M_PI;

    }
    catch (std::exception& e)
    {
        std::string errorMessage = e.what();
        ROS_WARN("Cannot read base joint states: \n %s", errorMessage.c_str());
    }

    // ensure that all joint values will be sent at the same time
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

    if(!controller_.is_enabled)
    {
        controller_command_ = velocity_command_;
    }
}

//########## UPDATE CONTROLLER #########################################################################################
void YoubotBaseInterface::updateController()
{
    if(base_ == NULL)
        return;

    if(!controller_.is_enabled)
    {
        return;
    }

    // Initialisierung

    double uP_x=0;
    double uP_y=0;
    double uP_z=0;

    double uD_x=0;
    double uD_y=0;
    double uD_z=0;


    double error[3];
    double anti_windup[3] = { 0.0, 0.0, 0.0 };

    double ta = delta_t_;

    // threshold Ta for safety reasons
    //    if (ta > 4.0 / config_->frequency)
    //        ta = 1.0 / config_->frequency;

    // Fehler berechnen

    error[0] = velocity_command_.linear.x - odom_msg_.twist.twist.linear.x;
    error[1] = velocity_command_.linear.y - odom_msg_.twist.twist.linear.y;
    error[2] = velocity_command_.angular.z - odom_msg_.twist.twist.angular.z;

    // P-Anteil

    uP_x=controller_.C_P[0]*error[0];
    uP_y=controller_.C_P[1]*error[1];
    uP_z=controller_.C_P[2]*error[2];

    // I-Anteil

    controller_.C_IM[0]=controller_.C_IM[0]+controller_.C_I[0]*(ta/2)*(controller_.error_alt[0]+error[0]);
    controller_.C_IM[1]=controller_.C_IM[1]+controller_.C_I[1]*(ta/2)*(controller_.error_alt[1]+error[1]);
    controller_.C_IM[2]=controller_.C_IM[2]+controller_.C_I[2]*(ta/2)*(controller_.error_alt[2]+error[2]);

    // Begrenzung des I-Anteils

    if(controller_.C_IM[0]>controller_.iSatUpper[0]) controller_.C_IM[0]=controller_.iSatUpper[0];
    if(controller_.C_IM[1]>controller_.iSatUpper[1]) controller_.C_IM[1]=controller_.iSatUpper[1];
    if(controller_.C_IM[2]>controller_.iSatUpper[2]) controller_.C_IM[2]=controller_.iSatUpper[2];

    if(controller_.C_IM[0]<controller_.iSatLower[0]) controller_.C_IM[0]=controller_.iSatLower[0];
    if(controller_.C_IM[1]<controller_.iSatLower[1]) controller_.C_IM[1]=controller_.iSatLower[1];
    if(controller_.C_IM[2]<controller_.iSatLower[2]) controller_.C_IM[2]=controller_.iSatLower[2];

    // D-Anteil

    controller_.C_DM[0] = controller_.C_DM[0]
            +(ta/2)*((controller_.C_D[0]*controller_.error_alt[0]-controller_.C_DM[0])*controller_.C_N[0]
            +(controller_.C_D[0]*error[0]-controller_.C_DM[0])*controller_.C_N[0]);
    controller_.C_DM[1] = controller_.C_DM[1]
            +(ta/2)*((controller_.C_D[1]*controller_.error_alt[1]-controller_.C_DM[1])*controller_.C_N[1]
            +(controller_.C_D[1]*error[1]-controller_.C_DM[1])*controller_.C_N[1]);
    controller_.C_DM[2] = controller_.C_DM[2]
            +(ta/2)* ((controller_.C_D[2]*controller_.error_alt[2]-controller_.C_DM[2])*controller_.C_N[2]
            +(controller_.C_D[2]*error[2]-controller_.C_DM[2])*controller_.C_N[2]);

    // Begrenzung des D-Filters

    if(controller_.C_DM[0]>controller_.dSatUpper[0]) controller_.C_DM[0]=controller_.dSatUpper[0];
    if(controller_.C_DM[1]>controller_.dSatUpper[1]) controller_.C_DM[1]=controller_.dSatUpper[1];
    if(controller_.C_DM[2]>controller_.dSatUpper[2]) controller_.C_DM[2]=controller_.dSatUpper[2];

    if(controller_.C_DM[0]<controller_.dSatLower[0]) controller_.C_DM[0]=controller_.dSatLower[0];
    if(controller_.C_DM[1]<controller_.dSatLower[1]) controller_.C_DM[1]=controller_.dSatLower[1];
    if(controller_.C_DM[2]<controller_.dSatLower[2]) controller_.C_DM[2]=controller_.dSatLower[2];

    uD_x = (controller_.C_D[0]*error[0]-controller_.C_DM[0])*controller_.C_N[0];
    uD_y = (controller_.C_D[1]*error[1]-controller_.C_DM[1])*controller_.C_N[1];
    uD_z = (controller_.C_D[2]*error[2]-controller_.C_DM[2])*controller_.C_N[2];

    // Geschwindigkeit stellen

    controller_command_.linear.x = uP_x + controller_.C_IM[0] + uD_x;
    controller_command_.linear.y = uP_y + controller_.C_IM[1] + uD_y;
    controller_command_.angular.z = uP_z + controller_.C_IM[2] + uD_z;


    // Stellgrößenbeschränkung

    // Thresholding and anti-windup vx-value
    if (controller_command_.linear.x > max_linear_vel_)
    {
        anti_windup[0] = max_linear_vel_ - controller_command_.linear.x;
        controller_command_.linear.x = max_linear_vel_;
    }
    if (controller_command_.linear.x < -max_linear_vel_)
    {
        anti_windup[0] = -max_linear_vel_ - controller_command_.linear.x;
        controller_command_.linear.x = -max_linear_vel_;
    }

    // Thresholding and anti-windup vy-value
    if (controller_command_.linear.y > max_linear_vel_)
    {
        anti_windup[1] = max_linear_vel_ - controller_command_.linear.y;
        controller_command_.linear.y = max_linear_vel_;
    }
    if (controller_command_.linear.y < -max_linear_vel_)
    {
        anti_windup[1] = -max_linear_vel_ - controller_command_.linear.y;
        controller_command_.linear.y = -max_linear_vel_;
    }

    // Thresholding and anti-windup vz-value
    if (controller_command_.angular.z > max_angular_vel_)
    {
        anti_windup[2] = max_angular_vel_ - controller_command_.angular.z;
        controller_command_.angular.z = max_angular_vel_;
    }
    if (controller_command_.angular.z < -max_angular_vel_)
    {
        anti_windup[2] = -max_angular_vel_ - controller_command_.angular.z;
        controller_command_.angular.z = -max_angular_vel_;
    }

    // calculate the error sum
    for (int n = 0; n < 3; n++)
        controller_.ctrl_sum[n] += error[n] + anti_windup[n];

    // apply a moving average filter on the calculated control values
    controller_command_.linear.x = controller_command_.linear.x *
            controller_.moving_average + last_controller_command_.linear.x * (1.0 - controller_.moving_average);

    controller_command_.linear.y = controller_command_.linear.y *
            controller_.moving_average + last_controller_command_.linear.y * (1.0 - controller_.moving_average);

    controller_command_.angular.z = controller_command_.angular.z *
            controller_.moving_average + last_controller_command_.angular.z * (1.0 - controller_.moving_average);

    // Einen Schritt vorwärts: k -> k+1

    last_controller_command_ = controller_command_;
    controller_.error_alt[0] = error[0];
    controller_.error_alt[1] = error[1];
    controller_.error_alt[2] = error[2];

}

//########## WRITE COMMANDS ############################################################################################
bool YoubotBaseInterface::writeCommands()
{
    if(base_ == NULL)
        return false;

    if(motors_are_off_)
        return false;

    bool has_succeeded = true;

    // ensure that all joint values will be sent at the same time
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
    try
    {
        quantity<si::velocity> velocity_command_x = controller_command_.linear.x * meter_per_second;
        quantity<si::velocity> velocity_command_y = controller_command_.linear.y * meter_per_second;
        quantity<si::angular_velocity> velocity_command_theta = controller_command_.angular.z * radian_per_second;

        base_->setBaseVelocity(velocity_command_x, velocity_command_y, velocity_command_theta);

    }
    catch (std::exception& e)
    {
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base velocities: \n %s", errorMessage.c_str());
        has_succeeded = false;
    }

    // ensure that all joint values will be sent at the same time
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

    return has_succeeded;

}

//########## STOP ######################################################################################################
void YoubotBaseInterface::stop()
{
    delete base_;
    base_ = NULL;

    switch_off_server_.shutdown();
    switch_on_server_.shutdown();

    config_->has_base = false;
}

//########## SET VELOCITY ##############################################################################################
void YoubotBaseInterface::setVelocity(const geometry_msgs::Twist &velocity)
{
    velocity_command_ = velocity;

    // security thresholding for x-axis
    if (velocity_command_.linear.x > max_linear_vel_)
        velocity_command_.linear.x = max_linear_vel_;
    if (velocity_command_.linear.x < -max_linear_vel_)
        velocity_command_.linear.x = -max_linear_vel_;

    // security thresholding for y-axis
    if (velocity_command_.linear.y > max_linear_vel_)
        velocity_command_.linear.y = max_linear_vel_;
    if (velocity_command_.linear.y < -max_linear_vel_)
        velocity_command_.linear.y = -max_linear_vel_;

    // security thresholding for z-axis
    if (velocity_command_.angular.z > max_angular_vel_)
        velocity_command_.angular.z = max_angular_vel_;
    if (velocity_command_.angular.z < -max_angular_vel_)
        velocity_command_.angular.z = -max_angular_vel_;

    motors_are_off_ = false;
}

//########## SET VELOCITY ##############################################################################################
void YoubotBaseInterface::setVelocity(double v_x, double v_y, double v_theta)
{
    geometry_msgs::Twist vel;
    vel.linear.x = v_x;
    vel.linear.y = v_y;
    vel.angular.z = v_theta;

    setVelocity(vel);
}

//########## PUBLISH MESSAGES ##########################################################################################
void YoubotBaseInterface::publishMessages()
{
    odom_broadcaster_.sendTransform(odom_tf_);
    odometry_publisher_.publish(odom_msg_);
    joint_state_publisher_.publish(joint_state_msg_);
    //    current_publisher_.publish(baseJointCurrentMessage);
    //    torque_publisher_.publish(baseJointTorqueMessage);
}

//########## SWITCH OFF MOTORS #########################################################################################
bool YoubotBaseInterface::switchOffBaseMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if(base_ == NULL)
        return false;

    youbot::JointCurrentSetpoint current_stop_movement;
    current_stop_movement.current = 0.0 * ampere;
    try
    {
        youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
        base_->getBaseJoint(1).setData(current_stop_movement);
        base_->getBaseJoint(2).setData(current_stop_movement);
        base_->getBaseJoint(3).setData(current_stop_movement);
        base_->getBaseJoint(4).setData(current_stop_movement);
        youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);
        ROS_INFO("Base motors have been switched off.");
    }
    catch (std::exception& e)
    {
        std::string errorMessage = e.what();
        ROS_WARN("Cannot switch off the base motors: \n %s", errorMessage.c_str());
        return false;
    }

    motors_are_off_ = true;

    return true;
}

//########## SWITCH ON MOTORS ##########################################################################################
bool YoubotBaseInterface::switchOnBaseMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    velocity_command_.linear.x = 0;
    velocity_command_.linear.y = 0;
    velocity_command_.angular.z = 0;

    motors_are_off_ = false;

    return true;
}

//########## GET POSE ##################################################################################################
geometry_msgs::Pose2D YoubotBaseInterface::getPose()
{
    return current_pose_;
}
