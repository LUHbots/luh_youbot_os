/* *****************************************************************
 *
 * luh_youbot_manipulation
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
 ******************************************************************/

#ifndef LUH_YOUBOT_MANIPULATION_NODE_H
#define LUH_YOUBOT_MANIPULATION_NODE_H

#include <ros/ros.h>
#include "../module_base_class/manipulation_module.h"
#include <luh_youbot_msgs/GetArmPose.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <luh_youbot_interface/youbot_interface.h>
#include <std_srvs/Empty.h>

class ManipulationNode
{
public:
    ManipulationNode(ros::NodeHandle &node);
    ~ManipulationNode();

protected:
    ros::NodeHandle* node_;

    ros::Timer arm_timer_;
    ros::Timer base_timer_;

    tf::TransformListener tf_listener_;

    ros::ServiceServer get_pose_server_;
    ros::ServiceServer stop_arm_server_;
    ros::ServiceServer stop_base_server_;
    ros::ServiceServer stop_bot_server_;

    std::vector<ManipulationModule*> arm_modules_;
    std::vector<ManipulationModule*> base_modules_;

    boost::mutex ethercat_mutex_;

    double arm_frequency_;
    double base_frequency_;

    YoubotInterface *youbot_;

    bool use_standard_gripper_;
    bool use_vrep_simulation_;

    void stopBase();
    void stopArm();

    // arm timer callback
    void armTimerCallback(const ros::TimerEvent &evt);

    // base timer callback
    void baseTimerCallback(const ros::TimerEvent &evt);

    // service callbacks
    bool getPoseCallback(luh_youbot_msgs::GetArmPose::Request &req,
                         luh_youbot_msgs::GetArmPose::Response &res);
    bool stopArmCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool stopBaseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool stopBotCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

};

#endif // LUH_YOUBOT_MANIPULATION_NODE_H
