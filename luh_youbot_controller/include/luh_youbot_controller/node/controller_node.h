/* *****************************************************************
 *
 * luh_youbot_controller
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
 ******************************************************************/

#ifndef LUH_YOUBOT_CONTROLLER_NODE_H
#define LUH_YOUBOT_CONTROLLER_NODE_H

#include <ros/ros.h>
#include "../module_base_class/controller_module.h"
#include <luh_youbot_msgs/GetArmPose.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <luh_youbot_driver_api/youbot_interface.h>
#include <std_srvs/Empty.h>

class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle &node);
    ~ControllerNode();

protected:
    ros::NodeHandle* node_;

    ros::Timer arm_timer_;
    ros::Timer base_timer_;

    tf::TransformListener tf_listener_;

    ros::ServiceServer get_pose_server_;
    ros::ServiceServer stop_arm_server_;
    ros::ServiceServer stop_base_server_;
    ros::ServiceServer stop_bot_server_;
    ros::ServiceServer enable_ramp_server_;
    ros::ServiceServer disable_ramp_server_;

    std::vector<ControllerModule*> arm_modules_;
    std::vector<ControllerModule*> base_modules_;

    boost::mutex ethercat_mutex_;

    double arm_frequency_;
    double base_frequency_;

    YoubotInterface *youbot_;

    bool use_standard_gripper_;
    bool use_vrep_simulation_;
    bool use_gazebo_simulation_;

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
    bool enableRampCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool disableRampCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

};

#endif // LUH_YOUBOT_CONTROLLER_NODE_H
