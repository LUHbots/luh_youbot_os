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

#ifndef LUH_YOUBOT_CONTROLLER_MODULE_GRIPPER_H
#define LUH_YOUBOT_CONTROLLER_MODULE_GRIPPER_H

#include "../module_base_class/controller_module.h"
#include "actionlib/server/simple_action_server.h"
#include "luh_youbot_msgs/GripObjectAction.h"
#include <luh_youbot_msgs/SetGripperAction.h>
#include <std_msgs/Float32.h>
#include "luh_youbot_msgs/GripCheck.h"

class ModuleGripper : public ControllerModule
{
public:
    typedef actionlib::SimpleActionServer<luh_youbot_msgs::GripObjectAction> GripObjectServer;
    typedef actionlib::SimpleActionServer<luh_youbot_msgs::SetGripperAction> SetGripperServer;

    ModuleGripper();
    ~ModuleGripper();

protected:

    bool activated_;
    bool gripping_object_;
    bool first_call_;
    bool publish_only_once_;
    double static_grip_force_;

    bool gripper_is_opening_;

    int gripper_update_counter_;


    double gripping_duration_;

    double current_gripper_width_;
    ros::Time start_time_;

    std::map<std::string, double> object_width_;
    GripObjectServer* grip_object_server_;
    SetGripperServer* set_gripper_server_;
    ros::ServiceServer grip_check_server_;

    ros::Subscriber gripper_subscriber_;
    double goal_width_;
    double min_gripper_width_;
    double max_gripper_width_;
    double gripper_velocity_;

    void init();
    void activate();
    void deactivate();
    void update();
    void emergencyStop();

    bool loadObjectWidth();
    void gripObjectCallback();
    void setGripperCallback();
    bool gripCheckCallback(luh_youbot_msgs::GripCheck::Request &req, luh_youbot_msgs::GripCheck::Response &res);

    void gripperMsgCallback(const std_msgs::Float32::ConstPtr &msg);
};

#endif // MODULE_GRIPPER_H
