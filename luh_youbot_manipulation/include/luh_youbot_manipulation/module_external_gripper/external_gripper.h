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

#ifndef LUH_YOUBOT_MANIPULATION_MODULE_EXTERNAL_GRIPPER_H
#define LUH_YOUBOT_MANIPULATION_MODULE_EXTERNAL_GRIPPER_H

#include "../module_base_class/manipulation_module.h"
#include "actionlib/server/simple_action_server.h"
#include "luh_youbot_msgs/GripObjectAction.h"
#include <luh_youbot_msgs/SetGripperAction.h>
#include <control_msgs/GripperCommand.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

class ModuleExternalGripper : public ManipulationModule
{
public:
    typedef actionlib::SimpleActionServer<luh_youbot_msgs::GripObjectAction> GripObjectServer;
    typedef actionlib::SimpleActionServer<luh_youbot_msgs::SetGripperAction> SetGripperServer;

    ModuleExternalGripper();
    ~ModuleExternalGripper();

protected:

    enum ControlMode
    {
        POSITION,
        FORCE,
        NONE
    }control_mode_;

    bool activated_;

    double grip_force_;
    double current_position_;
    double current_force_;

    double position_tolerance_;
    double force_tolerance_;

    double current_gripper_width_;
    ros::Time start_time_;

    GripObjectServer* grip_object_server_;
    SetGripperServer* set_gripper_server_;
    ros::Subscriber gripper_pos_cmd_subscriber_;
    ros::Subscriber gripper_state_subscriber_;
    ros::Publisher gripper_command_publisher_;

//    double goal_width_;
    double min_gripper_width_;
    double max_gripper_width_;

    control_msgs::GripperCommand gripper_command_;

    void init();
    void activate();
    void deactivate();
    void update();
    void emergencyStop();

    void gripObjectCallback();
    void setGripperCallback();

    void gripperMsgCallback(const std_msgs::Float32::ConstPtr &msg);
    void jointstateCallback(const sensor_msgs::JointState &state);
};

#endif // MODULE_EXTERNAL_GRIPPER_H
