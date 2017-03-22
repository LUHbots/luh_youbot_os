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

#ifndef LUH_YOUBOT_GAZEBO_BASE_INTERFACE_H
#define LUH_YOUBOT_GAZEBO_BASE_INTERFACE_H

//#include <sensor_msgs/JointState.h>
//#include <nav_msgs/Odometry.h>
//#include <std_srvs/Empty.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <geometry_msgs/Pose2D.h>
#include <luh_youbot_driver_api/base_interface.h>

class YoubotBaseGazeboInterface : public YoubotBaseInterface
{
public:
    YoubotBaseGazeboInterface(std::string name, YoubotConfiguration &config);
    ~YoubotBaseGazeboInterface();
    virtual void initialise();
    virtual void readState();
    virtual void updateController();
    virtual bool writeCommands();
    virtual void stop();
    virtual void publishMessages();

    virtual bool isInitialised(){return is_initialised_;}

protected:

    ros::Subscriber odom_subscriber_;
    ros::Publisher cmd_vel_publisher_;
    ros::Subscriber joint_state_subscriber_;
    std::vector<std::string> joint_names_;

    bool is_initialised_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state);
};
#endif // LUH_YOUBOT_GAZEBO_BASE_INTERFACE_H
