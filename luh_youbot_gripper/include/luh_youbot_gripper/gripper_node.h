#ifndef LUH_GRIPPER_NODE_H
#define LUH_GRIPPER_NODE_H

#include "luh_youbot_gripper/gripper.h"
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommand.h>
#include "luh_youbot_gripper/GripCheck.h"

//#include <iostream>
#include <fstream>


namespace luh_youbot_gripper
{
typedef actionlib::SimpleActionServer<luh_youbot_msgs::GripObjectAction> GripObjectServer;
typedef actionlib::SimpleActionServer<luh_youbot_msgs::SetGripperAction> SetGripperServer;


class GripperNode
{
public:
    GripperNode(ros::NodeHandle &node);


protected:
    ros::NodeHandle* node_;

    Gripper gripper_;

    GripObjectServer* grip_object_server_;
    SetGripperServer* set_gripper_server_;
    ros::Subscriber gripper_pos_cmd_subscriber_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher usb_voltage_publisher_;
    ros::Timer timer_;
    ros::ServiceServer grip_check_server_;

    sensor_msgs::JointState joint_state_msg_;
    std_msgs::Float32 usb_voltage_msg_;
    double current_position_;
    double open_force_;

    double max_effort_;
    double min_angle_;
    double max_angle_;

    double min_gripper_width_;
    double max_gripper_width_;
    double grip_force_;
    double position_tolerance_;
    double force_tolerance_;
    double static_grip_force_;

    void gripObjectCallback();
    void setGripperCallback();

    short positionToAngle(double position);
    void setGripperPosition(double position, bool is_relative=false);

    void gripperCommandCallback(const std_msgs::Float32::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &evt);
    bool gripCheckCallback(GripCheck::Request &req, GripCheck::Response &res);

    std::ofstream LogFile;
    bool use_voltage_log_;
};

}

#endif // GRIPPER_NODE_H
