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
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/

#include "luh_youbot_joy_teleop/joy_teleop_node.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

//########## CONSTRUCTOR ###############################################################################################
JoyTeleopNode::JoyTeleopNode():
    ros::NodeHandle(),
    defining_keys_(false),
    pressed_key_(-1),
    joint_vel_has_changed_(false),
    cart_vel_has_changed_(false),
    is_disabled_(false)
{
    // === ROS STUFF ===
    cartesian_velocity_publisher_ = this->advertise<luh_youbot_msgs::CartesianVector>("arm_1/cartesian_velocity", 1);
    joint_velocity_publisher_ = this->advertise<luh_youbot_msgs::JointVector>("arm_1/joint_velocity", 1);
    gripper_publisher_ = this->advertise<std_msgs::Float32>("arm_1/gripper_command", 1);    
    base_velocity_publisher_ = this->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    enable_server_ = this->advertiseService("youbot_joy/enable", &JoyTeleopNode::enableCallback, this);
    disable_server_ = this->advertiseService("youbot_joy/disable", &JoyTeleopNode::disableCallback, this);

    timer_ = this->createTimer(ros::Duration(0.1), &JoyTeleopNode::timerCallback, this);

    joy_subscriber_ = this->subscribe("/joy", 100, &JoyTeleopNode::joyCallback, this);

    // === PARAMS ===
    std::string default_dir = ros::package::getPath("luh_youbot_joy_teleop");
    default_dir.append("/cfg");
    this->param("luh_youbot_joy_teleop/key_config_dir", key_config_dir_, default_dir);
    if(*(key_config_dir_.end()-1) != '/')
        key_config_dir_.append("/");
    ROS_INFO("Key config directory: %s", key_config_dir_.c_str());

    this->param("luh_youbot_joy_teleop/key_config_file", key_config_file_, std::string("default_config.yaml"));
    this->param("luh_youbot_joy_teleop/gripper_velocity", gripper_velocity_, 0.02);
    this->param("luh_youbot_joy_teleop/arm_velocity_q1", arm_velocity_q1_, 0.4);
    this->param("luh_youbot_joy_teleop/arm_velocity_x", arm_velocity_x_, 0.04);
    this->param("luh_youbot_joy_teleop/arm_velocity_y", arm_velocity_y_, 0.04);
    this->param("luh_youbot_joy_teleop/arm_velocity_z", arm_velocity_z_, 0.04);
    this->param("luh_youbot_joy_teleop/arm_velocity_theta", arm_velocity_theta_, 0.15);
    this->param("luh_youbot_joy_teleop/arm_velocity_q5", arm_velocity_q5_, 0.4);
    this->param("luh_youbot_joy_teleop/base_velocity_x", base_velocity_x_, 0.4);
    this->param("luh_youbot_joy_teleop/base_velocity_y", base_velocity_y_, 0.4);
    this->param("luh_youbot_joy_teleop/base_velocity_theta", base_velocity_theta_, 0.4);
    this->param("luh_youbot_joy_teleop/gripper_max_pos", gripper_max_, 0.06);
    this->param("luh_youbot_joy_teleop/gripper_min_pos", gripper_min_, 0.0);
    this->param("luh_youbot_joy_teleop/youbot_has_arm", youbot_has_arm_, true);
    this->param("luh_youbot_joy_teleop/start_disabled", is_disabled_, false);

    this->param<std::string>("luh_youbot_joy_teleop/button_pose_1", button_pose_1_, "SEARCH_CENTER");
    this->param<std::string>("luh_youbot_joy_teleop/button_pose_2", button_pose_2_, "GRIP_CENTER");

    // === INIT VELOCITIES ===
    ee_velocity_.header.frame_id = "gripper_tip";

    // === YOUBOT ARM / GRIPPER ===
    if(youbot_has_arm_)
    {
        youbot_arm_.disableAllClients();
        youbot_arm_.enableClient(youbot_api::ActionClient::NAME_PLAN);
        youbot_arm_.init(*this);
        youbot_gripper_.init(*this);

        youbot_gripper_.setWidth(gripper_max_);
        gripper_state_.data = gripper_max_;
        gripper_velocity_command_ = 0;
    }

    loadKeyConfig();
}

//########## LOAD KEY CONFIG FILE ######################################################################################
bool JoyTeleopNode::loadKeyConfig()
{
    std::string filename = key_config_dir_;
    filename.append(key_config_file_);

    ROS_INFO("Loading file '%s'...", filename.c_str());

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(filename);
    }
    catch(YAML::BadFile &ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("Failed to laod file.");
        return false;
    }

    key_map_.assign(config.size(), NOT_ASSIGNED);

    int i=0;
    for(YAML::const_iterator it=config.begin();it!=config.end();++it)
    {
        YAML::const_iterator map_it = it->begin();
        std::string name = map_it->second.as<std::string>();

        if(name.compare("UP_DOWN") == 0)
        {
            key_map_[i] = UP_DOWN;
        }
        else if(name.compare("LEFT_RIGHT") == 0)
        {
            key_map_[i] = LEFT_RIGHT;
        }
        else if(name.compare("TURN_LEFT") == 0)
        {
            key_map_[i] = TURN_LEFT;
        }
        else if(name.compare("TURN_RIGHT") == 0)
        {
            key_map_[i] = TURN_RIGHT;
        }
        else if(name.compare("ARM_LEFT") == 0)
        {
            key_map_[i] = ARM_LEFT;
        }
        else if(name.compare("ARM_RIGHT") == 0)
        {
            key_map_[i] = ARM_RIGHT;
        }
        else if(name.compare("TO_HOME_POSE") == 0)
        {
            key_map_[i] = TO_HOME_POSE;
        }
        else if(name.compare("TO_SEARCH_POSE") == 0)
        {
            key_map_[i] = TO_SEARCH_POSE;
        }
        else if(name.compare("TO_GRIP_POSE") == 0)
        {
            key_map_[i] = TO_GRIP_POSE;
        }
        else if(name.compare("XY_MODE") == 0)
        {
            key_map_[i] = XY_MODE;
        }
        else if(name.compare("GRIP_MODE") == 0)
        {
            key_map_[i] = GRIP_MODE;
        }
        else if(name.compare("NOT_ASSIGNED") == 0)
        {
            key_map_[i] == NOT_ASSIGNED;
        }

        i++;
    }

    return true;
}

//########## TIMER CALLBACK ############################################################################################
void JoyTeleopNode::timerCallback(const ros::TimerEvent &evt)
{
    if(defining_keys_ || is_disabled_)
        return;

    base_velocity_publisher_.publish(base_velocity_);

    if(!youbot_has_arm_)
        return;

    if(gripper_velocity_command_ != 0)
    {
        double dt = (evt.current_real - evt.last_real).toSec();

        gripper_state_.data += gripper_velocity_command_ * dt;

        gripper_state_.data = std::max(gripper_min_, std::min(gripper_max_, (double)gripper_state_.data));

        gripper_publisher_.publish(gripper_state_);
    }
    else if(!youbot_arm_.isBusy())
    {        
        if(ee_velocity_.x == 0
                && ee_velocity_.y == 0
                && ee_velocity_.z == 0
                && ee_velocity_.theta == 0
                && ee_velocity_.q5 == 0
                && joint_velocity_.q1 == 0)
        {
            if(cart_vel_has_changed_)
            {
                ee_velocity_.header.stamp = ros::Time::now();
                cartesian_velocity_publisher_.publish(ee_velocity_);
            }
            else if(joint_vel_has_changed_)
            {
                joint_velocity_.header.stamp = ros::Time::now();
                joint_velocity_publisher_.publish(joint_velocity_);
            }
        }
        else if(joint_velocity_.q1 == 0)
        {
            ee_velocity_.header.stamp = ros::Time::now();
            cartesian_velocity_publisher_.publish(ee_velocity_);
        }
        else
        {
            joint_velocity_.header.stamp = ros::Time::now();
            joint_velocity_publisher_.publish(joint_velocity_);
        }
    }
}

//########## JOY CALLBACK ##############################################################################################
void JoyTeleopNode::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    if(is_disabled_)
        return;

    if(defining_keys_)
    {
        if(key_map_.empty())
            key_map_.assign(msg->axes.size() + msg->buttons.size(), NOT_ASSIGNED);
        int num_keys_pressed = 0;

        uint naxes = msg->axes.size();
        for(uint i=0; i<naxes; i++)
        {
            if(fabs(msg->axes[i]) > 0.1)
            {
                num_keys_pressed++;
                pressed_key_ = i;
            }
        }
        for(uint i=0; i<msg->buttons.size(); i++)
        {
            if(msg->buttons[i] != 0)
            {
                num_keys_pressed++;
                pressed_key_ = i + naxes;
            }
        }

//        if(num_keys_pressed != 1)
        if(num_keys_pressed == 0)
            pressed_key_ = -1;
    }
    else
    {
        // save last key state
        bool search_pose_was_pressed, grip_pose_was_pressed, home_pose_was_pressed;
        if(key_values_.empty())
        {
            search_pose_was_pressed = false;
            grip_pose_was_pressed = false;
            home_pose_was_pressed = false;
        }
        else
        {
            search_pose_was_pressed = key_values_[TO_SEARCH_POSE] != 0;
            grip_pose_was_pressed = key_values_[TO_GRIP_POSE] != 0;
            home_pose_was_pressed = key_values_[TO_HOME_POSE] != 0;
        }

        // get current key state
        uint naxes = msg->axes.size();
        for(uint i=0; i<naxes; i++)
        {
            int key = key_map_[i];
            key_values_[key] = msg->axes[i];
        }
        for(uint i=0; i<msg->buttons.size(); i++)
        {
            int key = key_map_[i + naxes];
            key_values_[key] = msg->buttons[i];
        }

        // check if one of the pose buttons was pressed
        if(!search_pose_was_pressed && key_values_[TO_SEARCH_POSE] != 0)
        {
            moveToSearchPose();
        }
        else if(!grip_pose_was_pressed && key_values_[TO_GRIP_POSE] != 0)
        {
            moveToGripPose();
        }
        else if(!home_pose_was_pressed && key_values_[TO_HOME_POSE] != 0)
        {
            moveToHomePose();
        }

//        std::cout << "key values:" << std::endl;
//        for(std::map<int,double>::const_iterator it = key_values_.begin(); it != key_values_.end(); it++)
//        {
//            std::cout << it->first << ": " << it->second << std::endl;
//        }

        // set velocities
        double old_q1 = joint_velocity_.q1;
        joint_velocity_.q1 = (key_values_[ARM_RIGHT] - key_values_[ARM_LEFT]) * arm_velocity_q1_;
        joint_vel_has_changed_ = (old_q1 != joint_velocity_.q1);


        double old_x = ee_velocity_.x;
        double old_y = ee_velocity_.y;
        double old_z = ee_velocity_.z;
        double old_theta = ee_velocity_.theta;
        double old_q5 = ee_velocity_.q5;

        if(key_values_[XY_MODE] != 0)
        {
            // youbot
            ee_velocity_.x = 0;
            ee_velocity_.y = arm_velocity_y_ * key_values_[LEFT_RIGHT];
            ee_velocity_.z = arm_velocity_z_ * key_values_[UP_DOWN];
            ee_velocity_.q5 = arm_velocity_q5_ * (key_values_[TURN_LEFT] - key_values_[TURN_RIGHT]);

            // vrep
//            ee_velocity_.x = arm_velocity_x_ * key_values_[UP_DOWN];
//            ee_velocity_.y = -arm_velocity_y_ * key_values_[LEFT_RIGHT];
//            ee_velocity_.z = 0;
//            ee_velocity_.q5 = arm_velocity_q5_ * (key_values_[TURN_LEFT] - key_values_[TURN_RIGHT]);

            ee_velocity_.theta = 0;

            gripper_velocity_command_ = 0;
            base_velocity_.linear.x = 0;
            base_velocity_.linear.y = 0;
            base_velocity_.angular.z = 0;
        }
        else if(key_values_[GRIP_MODE] != 0)
        {
            // youbot
            ee_velocity_.x = arm_velocity_x_ * key_values_[UP_DOWN];
            ee_velocity_.y = 0;
            ee_velocity_.z = 0;
            ee_velocity_.q5 = 0;

            // vrep
//            ee_velocity_.x = 0;
//            ee_velocity_.y = 0;
//            ee_velocity_.z = arm_velocity_z_ * key_values_[UP_DOWN];
//            ee_velocity_.q5 = 0;

            ee_velocity_.theta = arm_velocity_theta_ * (key_values_[TURN_RIGHT] - key_values_[TURN_LEFT]);

            gripper_velocity_command_ = -gripper_velocity_ * key_values_[LEFT_RIGHT];
            base_velocity_.linear.x = 0;
            base_velocity_.linear.y = 0;
            base_velocity_.angular.z = 0;
        }
        else
        {
            ee_velocity_.x = 0;
            ee_velocity_.y = 0;
            ee_velocity_.q5 = 0;
            ee_velocity_.theta = 0;
            ee_velocity_.z = 0;
            gripper_velocity_command_ = 0;
            base_velocity_.linear.x = base_velocity_x_ * key_values_[UP_DOWN];
            base_velocity_.linear.y = base_velocity_y_ * key_values_[LEFT_RIGHT];
            base_velocity_.angular.z = base_velocity_theta_ * (key_values_[TURN_LEFT] - key_values_[TURN_RIGHT]);
        }

        cart_vel_has_changed_ =
                (old_x != ee_velocity_.x
                || old_y != ee_velocity_.y
                || old_z != ee_velocity_.z
                || old_theta != ee_velocity_.theta
                || old_q5 != ee_velocity_.q5);


    }
}

//########## WAIT FOR KEY ##############################################################################################
int JoyTeleopNode::waitForKey()
{
    while(pressed_key_ == -1)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    int key = pressed_key_;

    while(pressed_key_ != -1)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    return key;
}

//########## SAVE KEY CONFIG ###########################################################################################
void JoyTeleopNode::saveKeyConfig(std::string filename)
{
    std::string path = key_config_dir_;
    path.append(filename);

    std::ofstream fout(path.c_str());

    YAML::Node seq_node;
    for(uint i=0; i<key_map_.size(); i++)
    {
        std::string name;
        if(key_map_[i] == UP_DOWN)
            name = "UP_DOWN";
        else if(key_map_[i] == LEFT_RIGHT)
            name = "LEFT_RIGHT";
        else if(key_map_[i] == TURN_LEFT)
            name = "TURN_LEFT";
        else if(key_map_[i] == TURN_RIGHT)
            name = "TURN_RIGHT";
        else if(key_map_[i] == ARM_LEFT)
            name = "ARM_LEFT";
        else if(key_map_[i] == ARM_RIGHT)
            name = "ARM_RIGHT";
        else if(key_map_[i] == TO_HOME_POSE)
            name = "TO_HOME_POSE";
        else if(key_map_[i] == TO_SEARCH_POSE)
            name = "TO_SEARCH_POSE";
        else if(key_map_[i] == TO_GRIP_POSE)
            name = "TO_GRIP_POSE";
        else if(key_map_[i] == XY_MODE)
            name = "XY_MODE";
        else if(key_map_[i] == GRIP_MODE)
            name = "GRIP_MODE";
        else if(key_map_[i] == NOT_ASSIGNED)
            name = "NOT_ASSIGNED";

        std::stringstream ss;
        ss << "Key " << i;
        seq_node[i][ss.str()] = name;
    }

    fout << seq_node << std::endl;

    std::cout << "Saved config to: " << path << std::endl;
}

//########## DEFINE KEYS ###############################################################################################
void JoyTeleopNode::defineKeys()
{
    defining_keys_ = true;

    key_map_.clear();

    std::cout << "Press UP/DOWN axis." << std::endl;
    int key = waitForKey();
    key_map_[key] = UP_DOWN;

    std::cout << "Press LEFT/RIGHT axis." << std::endl;
    key = waitForKey();
    key_map_[key] = LEFT_RIGHT;

    std::cout << "Press TURN LEFT key." << std::endl;
    key = waitForKey();
    key_map_[key] = TURN_LEFT;

    std::cout << "Press TURN RIGHT key." << std::endl;
    key = waitForKey();
    key_map_[key] = TURN_RIGHT;

    std::cout << "Press TURN ARM LEFT key." << std::endl;
    key = waitForKey();
    key_map_[key] = ARM_LEFT;
    std::cout << "Press TURN ARM RIGHT key." << std::endl;
    key = waitForKey();
    key_map_[key] = ARM_RIGHT;

    std::cout << "Press TO HOME POSE key." << std::endl;
    key = waitForKey();
    key_map_[key] = TO_HOME_POSE;

    std::cout << "Press TO SEARCH POSE key." << std::endl;
    key = waitForKey();
    key_map_[key] = TO_SEARCH_POSE;

    std::cout << "Press TO GRIP POSE key." << std::endl;
    key = waitForKey();
    key_map_[key] = TO_GRIP_POSE;

    std::cout << "Press XY MODE key." << std::endl;
    key = waitForKey();
    key_map_[key] = XY_MODE;

    std::cout << "Press GRIP_MODE key." << std::endl;
    key = waitForKey();
    key_map_[key] = GRIP_MODE;

    std::string ans;
    std::cout << "Save key config? [y/n]: ";
    std::cin >> ans;
    if(!ans.empty() && (ans[0] == 'y' || ans[0]=='Y'))
    {
        std::cout << "Enter a name: ";
        std::cin >> ans;
        ans.append(".yaml");
        saveKeyConfig(ans);
    }
    else
    {
        std::cout << "Key config not saved." << std::endl;
    }

    defining_keys_ = false;
}

//########## MOVE TO SEARCH POSE #######################################################################################
void JoyTeleopNode::moveToSearchPose()
{    
    if(youbot_has_arm_)
    {
        ROS_INFO("Moving to %s...", button_pose_1_.c_str());
        youbot_arm_.moveToPose(button_pose_1_);
    }
}

//########## MOVE TO GRIP POSE #########################################################################################
void JoyTeleopNode::moveToGripPose()
{
    if(youbot_has_arm_)
    {
        ROS_INFO("Moving to %s...", button_pose_2_.c_str());
        youbot_arm_.moveToPose(button_pose_2_);
    }
}

//########## MOVE TO HOME POSE #########################################################################################
void JoyTeleopNode::moveToHomePose()
{
    if(youbot_has_arm_)
    {
        ROS_INFO("Moving to home pose...");
        youbot_arm_.moveToPose("HOME");
    }
}

//########## ENABLE SERVICE ############################################################################################
bool JoyTeleopNode::enableCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    is_disabled_ = false;

    return true;
}

//########## DISABLE SERVICE ###########################################################################################
bool JoyTeleopNode::disableCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    is_disabled_ = true;

    // set all velocities to zero
    ee_velocity_.x = 0;
    ee_velocity_.y = 0;
    ee_velocity_.q5 = 0;
    ee_velocity_.theta = 0;
    ee_velocity_.z = 0;
    gripper_velocity_command_ = 0;
    base_velocity_.linear.x = 0;
    base_velocity_.linear.y = 0;
    base_velocity_.angular.z = 0;

    ee_velocity_.header.stamp = ros::Time::now();
    cartesian_velocity_publisher_.publish(ee_velocity_);
    base_velocity_publisher_.publish(base_velocity_);

    return true;
}

//########## MAIN ######################################################################################################
int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_joy_teleop");

    JoyTeleopNode node;

    if(argc == 2)
    {
        if(std::string(argv[1]).compare("--assign-buttons") == 0)
            node.defineKeys();
        else
            argc++;
    }
    if(argc > 2)
    {
        std::cout << "usage: youbot_teleop [--assign-buttons]" << std::endl;
        return 0;
    }

    ROS_INFO("ROS is spinning...");
    ros::spin();

    return 0;
}


