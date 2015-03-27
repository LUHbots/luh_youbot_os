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

#ifndef LUH_YOUBOT_VREP_ARM_INTERFACE_H
#define LUH_YOUBOT_VREP_ARM_INTERFACE_H

#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <luh_youbot_interface/arm_interface.h>

class YoubotArmVrepInterface : public YoubotArmInterface
{
public:
    YoubotArmVrepInterface(std::string name, YoubotConfiguration &config);
    ~YoubotArmVrepInterface();
    virtual void initialise();
    virtual void readState();
    virtual bool writeCommands();
    virtual void stop();
    virtual void publishMessages();

    virtual bool isInitialised(){return is_initialised_;}

    virtual bool securityCheck();

protected:

    std::vector<ros::Publisher> pub_arm_position_command_;
    std::vector<ros::Subscriber> sub_arm_joint_states_;
    ros::Subscriber sub_gripper_l_joint_states_;
    ros::Subscriber sub_gripper_r_joint_states_;
    ros::Publisher pub_gripper_l_position_command_;
    ros::Publisher pub_gripper_r_position_command_;

    bool is_initialised_;

    void armJointStateCallback(const sensor_msgs::JointState& joint_state);
    void gripperJointStateCallback(const sensor_msgs::JointState& joint_state);
};

#endif // LUH_YOUBOT_ARM_INTERFACE_H
