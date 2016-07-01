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

#include "luh_youbot_controller/module_gripper/gripper.h"
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
ModuleGripper::ModuleGripper(): ControllerModule(),
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
    node_->param("module_gripper/static_grip_force", static_grip_force_, 30.0);
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


    // === SERVICE SERVER ===
    grip_check_server_ = node_->advertiseService("gripper/check", &ModuleGripper::gripCheckCallback, this);

    //Set Gripper Update Counter
    gripper_update_counter_=0;
    gripper_is_opening_=false;

    ROS_INFO("Gripper Module initialised.");
}

//########## LOAD OBJECT WIDTH FILE ####################################################################################
bool ModuleGripper::loadObjectWidth()
{    
    std::string filename = ros::package::getPath("luh_youbot_controller");
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



        //check the gripper effort feedback

        double current_force = fabs(youbot_->arm()->getGripperEffort());

        bool object_is_inside_the_gripper=false;
        object_is_inside_the_gripper= current_force > static_grip_force_*3.0;
        if(current_force>static_grip_force_*3.0)
        {
            gripper_update_counter_++;
//            ROS_INFO("Gripper Update, current forse is: %f", current_force);
//            ROS_INFO("Gripper Update, object_is_inside_the_gripper is : %i",object_is_inside_the_gripper);
        }


        if(delta_t < gripping_duration_)
        {
            if(first_call_)
            {
                youbot_->arm()->setGripperPosition(goal_width_);
                first_call_ = false;
                gripper_update_counter_=0;
            }
        }

        if(delta_t > gripping_duration_ || (object_is_inside_the_gripper && gripper_update_counter_>10 && !gripper_is_opening_) || (gripper_is_opening_ && delta_t*5.0 > gripping_duration_) )
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

        if (new_goal_width>goal_width_)
        {
            gripper_is_opening_=true;
        }
        else{
            gripper_is_opening_=false;
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

    luh_youbot_msgs::SetGripperGoal::ConstPtr goal = set_gripper_server_->acceptNewGoal();
    double new_goal_width = goal->gripper_width;
    if(goal->is_relative)
    {
        ROS_WARN("Relative goals are not implemented for the standard gripper.");
        set_gripper_server_->setAborted();
        return;
    }

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

    if (new_goal_width>goal_width_)
    {
        gripper_is_opening_=true;
    }
    else{
        gripper_is_opening_=false;
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

    if (msg->data>goal_width_)
    {
        gripper_is_opening_=true;
    }
    else{
        gripper_is_opening_=false;
    }


    goal_width_ = msg->data;
}

//########## CALLBACK: CHECK GRIPPER  ########################################################################
bool ModuleGripper::gripCheckCallback(luh_youbot_msgs::GripCheck::Request &req, luh_youbot_msgs::GripCheck::Response &res)
{

    double current_force = fabs(youbot_->arm()->getGripperEffort());

    res.has_object = current_force > static_grip_force_;


    return true;
}
