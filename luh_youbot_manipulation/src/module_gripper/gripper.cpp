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

#include "luh_youbot_manipulation/module_gripper/gripper.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "ros/package.h"
#include <boost/units/io.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/units/systems/si.hpp>

//########## CONSTRUCTOR ###############################################################################################
ModuleGripper::ModuleGripper(): ManipulationModule(),
    grip_object_server_(NULL),
    set_gripper_server_(NULL)
{
}

//########## DESTRUCTOR ###############################################################################################
ModuleGripper::~ModuleGripper()
{
    delete grip_object_server_;
    delete set_gripper_server_;
}

//########## INITIALIZATION ############################################################################################
void ModuleGripper::init()
{
    ROS_INFO("Initialising Gripper Module...");
    status_ = STATUS_IDLE;    

    // === PARAMETERS ===
    activated_ = true;
    gripping_object_ = false;
    first_call_ = false;
    publish_only_once_ = false;
    if(!loadObjectWidth())
            ROS_ERROR("Could not load file 'object_width.yaml'.");
    node_->param("module_gripper/min_gripper_width", min_gripper_width_, 0.0);
    node_->param("module_gripper/max_gripper_width", max_gripper_width_, 0.06);
    node_->param("module_gripper/gripper_velocity", gripper_velocity_, 0.02);
    goal_width_ = HUGE_VAL; // huge val means unknown

    // === ACTION SERVERS ===
    grip_object_server_ = new GripObjectServer(*node_, "arm_1/grip_object", false);
    grip_object_server_->registerGoalCallback(boost::bind(&ModuleGripper::gripObjectCallback, this));
    grip_object_server_->start();
    set_gripper_server_ = new SetGripperServer(*node_, "arm_1/set_gripper", false);
    set_gripper_server_->registerGoalCallback(boost::bind(&ModuleGripper::setGripperCallback, this));
    set_gripper_server_->start();

    // === SUBSCRIBER ===
    gripper_subscriber_ = node_->subscribe("arm_1/gripper_command", 1, &ModuleGripper::gripperMsgCallback, this);

    ROS_INFO("Gripper Module initialised.");
}

//########## LOAD OBJECT WIDTH FILE ####################################################################################
bool ModuleGripper::loadObjectWidth()
{    
    std::string filename = ros::package::getPath("luh_youbot_manipulation");
    filename.append("/cfg/module_gripper/object_width.yaml");

    ROS_INFO("Loading file '%s'...", filename.c_str());

    YAML::Node config = YAML::LoadFile(filename);
    for(YAML::const_iterator it=config.begin();it!=config.end();++it)
    {
        YAML::const_iterator map_it = it->begin();
        std::string name = map_it->first.as<std::string>();
        double value = map_it->second.as<double>(0);

        object_width_[name] = value;
    }

    return true;
}

//########## UPDATE ####################################################################################################
void ModuleGripper::update()
{
    if(!activated_)
        return;

    if(gripping_object_)
    {
        if(publish_only_once_)
        {
            youbot_->arm()->setGripperPosition(goal_width_);
            gripping_object_ = false;
            return;
        }

        double delta_t = (ros::Time::now() - start_time_).toSec();

        if(delta_t < gripping_duration_)
        {
            if(first_call_)
            {
                youbot_->arm()->setGripperPosition(goal_width_);
                first_call_ = false;
            }
        }
        else
        {
            gripping_object_ = false;
            gripper_is_busy_ = false;
            ROS_INFO("Gripper action finished.");

            if(grip_object_server_->isActive())
                grip_object_server_->setSucceeded();
            else if(set_gripper_server_->isActive())
                set_gripper_server_->setSucceeded();
        }
    }

}

//########## ACTIVATE ##################################################################################################
void ModuleGripper::activate()
{
    activated_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleGripper::deactivate()
{
    activated_ = false;
}

//########## EMERGENCY STOP ############################################################################################
void ModuleGripper::emergencyStop()
{
    if(grip_object_server_->isActive())
    {
        grip_object_server_->setAborted();
    }
    if(set_gripper_server_->isActive())
    {
        set_gripper_server_->setAborted();
    }

    gripper_is_busy_ = false;
    gripping_object_ = false;
}

//########## CALLBACK: GRIP OBJECT #####################################################################################
void ModuleGripper::gripObjectCallback()
{
    boost::mutex::scoped_lock lock(gripper_mutex_);

    ROS_INFO("==== Module Gripper ====");

    std::string name = grip_object_server_->acceptNewGoal()->object_name;

    if(object_width_.find(name) != object_width_.end())
    {
        double new_goal_width = object_width_[name];

        publish_only_once_ = false;
        gripping_object_ = true;
        start_time_ = ros::Time::now();
        gripper_is_busy_ = true;
        first_call_ = true;

        if(goal_width_ > max_gripper_width_)
        {
            // set max duration when current gripper state is unknown
            gripping_duration_ = fabs(max_gripper_width_ - min_gripper_width_) / gripper_velocity_;
        }
        else
        {
            gripping_duration_ = fabs(goal_width_ - new_goal_width) / gripper_velocity_;
        }
        goal_width_ = new_goal_width;

        ROS_INFO("Gripper command received.");
    }
    else
    {
        ROS_ERROR("Request to grip unknown object '%s'.", name.c_str());
        grip_object_server_->setAborted();
    }
}

//########## CALLBACK: SET GRIPPER (ACTION) ############################################################################
void ModuleGripper::setGripperCallback()
{
    boost::mutex::scoped_lock lock(gripper_mutex_);

    ROS_INFO("==== Module Gripper ====");

    double new_goal_width = set_gripper_server_->acceptNewGoal()->gripper_width;

    if(new_goal_width > max_gripper_width_ || new_goal_width < min_gripper_width_)
    {
        ROS_ERROR("Requested gripper width of %f is out of range.", new_goal_width);

        set_gripper_server_->setAborted();

        return;
    }

    publish_only_once_ = false;
    gripping_object_ = true;
    start_time_ = ros::Time::now();
    gripper_is_busy_ = true;
    first_call_ = true;

    if(goal_width_ > max_gripper_width_)
    {
        // set max duration when current gripper state is unknown
        gripping_duration_ = fabs(max_gripper_width_ - min_gripper_width_) / gripper_velocity_;
    }
    else
    {
        gripping_duration_ = fabs(goal_width_ - new_goal_width) / gripper_velocity_;
    }
    goal_width_ = new_goal_width;

    ROS_INFO("Gripper command received.");
}

//########## CALLBACK: SET GRIPPER (SUBSCRIBER) ########################################################################
void ModuleGripper::gripperMsgCallback(const std_msgs::Float32::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(gripper_mutex_);

    if(msg->data > max_gripper_width_ || msg->data < min_gripper_width_)
    {
        ROS_ERROR("Requested gripper width of %f is out of range.", msg->data);
        return;
    }

    gripping_object_ = true;
    publish_only_once_ = true;
    goal_width_ = msg->data;
}
