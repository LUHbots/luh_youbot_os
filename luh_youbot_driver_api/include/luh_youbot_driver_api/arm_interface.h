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

#ifndef LUH_YOUBOT_ARM_INTERFACE_H
#define LUH_YOUBOT_ARM_INTERFACE_H

#include "luh_youbot_driver_api/common.h"
#include <youbot/YouBotManipulator.hpp>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include "luh_youbot_driver_api/arduino_gripper.h"

class YoubotArmInterface
{
public:
    YoubotArmInterface(std::string name, YoubotConfiguration &config);
    ~YoubotArmInterface();
    virtual void initialise(bool use_standard_gripper=true, bool use_luh_gripper_v3=false);
    virtual void readState();
    virtual bool writeCommands();
    virtual void stop();
    virtual void publishMessages();
    void setJointPositions(luh_youbot_kinematics::JointPosition positions);
    void setJointVelocities(luh_youbot_kinematics::JointVelocity velocities);
    void setJointTorques(luh_youbot_kinematics::JointVector torques);
    void setGripperPosition(double left, double right);
    void setGripperPosition(double width);
    luh_youbot_kinematics::JointPosition getJointPosition();
    luh_youbot_kinematics::JointVelocity getJointVelocity();
    luh_youbot_kinematics::JointVector getJointTorque();
    double getGripperEffort(){return gripper_effort_;}
    double getGripperVelocity(){return gripper_velocity_;}
    double getGripperPosition(){return gripper_position_;}

    virtual bool isInitialised(){return arm_ != NULL;}

    virtual int securityCheck();

    void enableRampGenerator(bool enable);

protected:

    ros::Publisher joint_state_publisher_;
    ros::ServiceServer switch_on_server_;
    ros::ServiceServer switch_off_server_;
    ros::ServiceServer calibrate_server_;

    YoubotConfiguration *config_;

    sensor_msgs::JointState joint_state_;

    luh_youbot_kinematics::JointPosition joint_position_;
    luh_youbot_kinematics::JointVelocity joint_velocity_;
    luh_youbot_kinematics::JointVector joint_torque_;
    double gripper_position_;
    double gripper_velocity_;
    double gripper_effort_;

    bool ramp_is_disabled_;
    std::string name_;

    youbot::YouBotManipulator *arm_;
    arduino_gripper* luh_gripper_v3_;
    int luh_gripper_v3_read_state_counter_;

    enum ControlMode{POSITION, VELOCITY, TORQUE} mode_;

    int arm_index_;
    std::string topic_prefix_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> gripper_finger_names_;

    std::vector<double> torque_command_;
    std::vector<double> velocity_command_;
    std::vector<double> position_command_;

    std::vector<double> gripper_command_;
    float luh_gripper_v3_position_command_;

    bool has_new_gripper_command_;
    bool has_new_arm_command_;

    const static unsigned int LEFT_FINGER_INDEX = 0;
    const static unsigned int RIGHT_FINGER_INDEX = 1;

    ros::Time current_sample_time_;
    ros::Time last_sample_time_;
    double delta_t_;

    luh_youbot_kinematics::JointVector max_effort_;
    double max_effort_max_duration_;
    std::vector<ros::Time> effort_watchdog_time_;

    bool use_standard_gripper_;
    bool use_luh_gripper_v3_;

    bool switchOffArmMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool switchOnArmMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool calibrateArmCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

};

#endif // LUH_YOUBOT_ARM_INTERFACE_H
