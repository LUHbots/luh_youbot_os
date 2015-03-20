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


#include "luh_youbot_manipulation/module_base_class/manipulation_module.h"

#include <ros/package.h>

#include <boost/units/io.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/units/systems/si.hpp>

using namespace luh_youbot_kinematics;


// static members
bool ManipulationModule::arm_is_busy_;
bool ManipulationModule::base_is_busy_;
bool ManipulationModule::gripper_is_busy_;
bool ManipulationModule::is_initialised_;
ros::NodeHandle* ManipulationModule::node_;
double ManipulationModule::arm_update_frequency_;
double ManipulationModule::base_update_frequency_;
tf::TransformListener* ManipulationModule::tf_listener_;
YoubotInterface* ManipulationModule::youbot_;
youbot_poses::PoseMap ManipulationModule::predefined_poses_;

//######################### CONSTRUCTOR ################################################################################
ManipulationModule::ManipulationModule()
{

}

//######################### INIT #######################################################################################
void ManipulationModule::initStatic(ros::NodeHandle *node, tf::TransformListener &tf_listener,
                                    YoubotInterface *youbot)
{
    node_ = node;
    tf_listener_ = &tf_listener;
    youbot_ = youbot;

    // === READ PREDEFINED POSES ===
    std::string filename;
    node->param("luh_youbot_manipulation/poses_file", filename, filename);

    ROS_INFO("Loading poses from %s", filename.c_str());
    predefined_poses_ = youbot_poses::read(filename);
    if(filename.empty())
        ROS_INFO("Loaded default poses.yaml from luh_youbot_poses.");
    else
        ROS_INFO("Loaded poses from: %s", filename.c_str());
}

//######################### SET UPDATE FREQUENCY #######################################################################
void ManipulationModule::setArmUpdateFrequency(double frequency)
{
    arm_update_frequency_ = frequency;
}

//######################### SET UPDATE FREQUENCY #######################################################################
void ManipulationModule::setBaseUpdateFrequency(double frequency)
{
    base_update_frequency_ = frequency;
}

//######################### BUSY FLAGS #################################################################################
void ManipulationModule::setArmIsBusy(bool arm_is_busy)
{
    arm_is_busy_ = arm_is_busy;
}
void ManipulationModule::setBaseIsBusy(bool base_is_busy)
{
    base_is_busy_ = base_is_busy;
}
void ManipulationModule::setGripperIsBusy(bool gripper_is_busy)
{
    gripper_is_busy_ = gripper_is_busy;
}
bool ManipulationModule::isArmBusy()
{
    return arm_is_busy_;
}
bool ManipulationModule::isBaseBusy()
{
    return base_is_busy_;
}
bool ManipulationModule::isGripperBusy()
{
    return gripper_is_busy_;
}
