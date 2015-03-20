/* *****************************************************************
 *
 * luh_youbot_joy_teleop
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

#ifndef LUH_YOUBOT_JOY_TELEOP_NODE_H
#define LUH_YOUBOT_JOY_TELEOP_NODE_H

#include <ros/ros.h>
#include <luh_youbot_manipulation_api/youbot_arm.h>
#include <luh_youbot_manipulation_api/youbot_gripper.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

class JoyTeleopNode : public ros::NodeHandle
{
public:
    JoyTeleopNode();

    void defineKeys();

protected:

    enum Key{
        LEFT_RIGHT,
        UP_DOWN,
        TURN_LEFT,
        TURN_RIGHT,
        ARM_LEFT,
        ARM_RIGHT,
        TO_SEARCH_POSE,
        TO_GRIP_POSE,
        TO_HOME_POSE,
        XY_MODE,
        GRIP_MODE,
        NOT_ASSIGNED
    };

    ros::Publisher cartesian_velocity_publisher_;
    ros::Publisher joint_velocity_publisher_;
    ros::Publisher gripper_publisher_;
    ros::Publisher base_velocity_publisher_;

    ros::Subscriber joy_subscriber_;

    ros::Timer timer_;

    manipulation_api::YoubotArm youbot_arm_;
    manipulation_api::YoubotGripper youbot_gripper_;

    ros::ServiceServer enable_server_;
    ros::ServiceServer disable_server_;

    bool defining_keys_;

    std::vector<int> key_map_;
    std::map<int, double> key_values_;
    int pressed_key_;

    std::string key_config_dir_;
    std::string key_config_file_;

    luh_youbot_msgs::CartesianVector ee_velocity_;
    luh_youbot_msgs::JointVector joint_velocity_;
    geometry_msgs::Twist base_velocity_;
    double gripper_velocity_command_;
    std_msgs::Float32 gripper_state_;

    double gripper_velocity_;
    double arm_velocity_q1_;
    double arm_velocity_x_;
    double arm_velocity_y_;
    double arm_velocity_z_;
    double arm_velocity_theta_;
    double arm_velocity_q5_;
    double base_velocity_x_;
    double base_velocity_y_;
    double base_velocity_theta_;
    double gripper_max_;
    double gripper_min_;

    bool youbot_has_arm_;

    bool joint_vel_has_changed_;
    bool cart_vel_has_changed_;

    bool is_disabled_;

    std::string button_pose_1_;
    std::string button_pose_2_;

    bool loadKeyConfig();

    int waitForKey();
    void saveKeyConfig(std::string filename);

    void timerCallback(const ros::TimerEvent &evt);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    void moveToSearchPose();
    void moveToGripPose();
    void moveToHomePose();

    bool enableCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool disableCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

};

#endif // JOY_TELEOP_NODE_H
