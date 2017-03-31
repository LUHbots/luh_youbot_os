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

#ifndef LUH_YOUBOT_GAZEBO_ARM_INTERFACE_H
#define LUH_YOUBOT_GAZEBO_ARM_INTERFACE_H

#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <luh_youbot_driver_api/arm_interface.h>

class YoubotArmGazeboInterface : public YoubotArmInterface
{
public:
    YoubotArmGazeboInterface(std::string name, YoubotConfiguration &config);
    ~YoubotArmGazeboInterface();
    virtual void initialise(bool use_standard_gripper=true, bool use_luh_gripper_v3=false);
    virtual void readState();
    virtual bool writeCommands();
    virtual void stop();
    virtual void publishMessages();

    virtual bool isInitialised(){return is_initialised_;}

    virtual int securityCheck();

protected:

    ros::Publisher position_command_publisher_;
    ros::Publisher velocity_command_publisher_;
    ros::Publisher torque_command_publisher_;

    bool is_initialised_;

    ros::Subscriber joint_state_subscriber_;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
};

#endif // LUH_YOUBOT_GAZEBO_ARM_INTERFACE_H
