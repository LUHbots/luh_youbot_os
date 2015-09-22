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


#include "luh_youbot_controller/module_base_class/controller_module.h"

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
bool ControllerModule::arm_is_busy_;
bool ControllerModule::base_is_busy_;
bool ControllerModule::gripper_is_busy_;
bool ControllerModule::is_initialised_;
ros::NodeHandle* ControllerModule::node_;
double ControllerModule::arm_update_frequency_;
double ControllerModule::base_update_frequency_;
tf::TransformListener* ControllerModule::tf_listener_;
YoubotInterface* ControllerModule::youbot_;
youbot_poses::PoseMap ControllerModule::predefined_poses_;
boost::mutex ControllerModule::arm_mutex_;
boost::mutex ControllerModule::base_mutex_;
boost::mutex ControllerModule::gripper_mutex_;

//######################### CONSTRUCTOR ################################################################################
ControllerModule::ControllerModule()
{

}

//######################### INIT #######################################################################################
void ControllerModule::initStatic(ros::NodeHandle *node, tf::TransformListener &tf_listener,
                                    YoubotInterface *youbot)
{
    node_ = node;
    tf_listener_ = &tf_listener;
    youbot_ = youbot;

    // === READ PREDEFINED POSES ===
    std::string filename;
    node->param("luh_youbot_controller/poses_file", filename, filename);

    ROS_INFO("Loading poses from %s", filename.c_str());
    predefined_poses_ = youbot_poses::read(filename);
    if(filename.empty())
        ROS_INFO("Loaded default poses.yaml from luh_youbot_poses.");
    else
        ROS_INFO("Loaded poses from: %s", filename.c_str());
}

//######################### SET UPDATE FREQUENCY #######################################################################
void ControllerModule::setArmUpdateFrequency(double frequency)
{
    arm_update_frequency_ = frequency;
}

//######################### SET UPDATE FREQUENCY #######################################################################
void ControllerModule::setBaseUpdateFrequency(double frequency)
{
    base_update_frequency_ = frequency;
}

//######################### BUSY FLAGS #################################################################################
void ControllerModule::setArmIsBusy(bool arm_is_busy)
{
    arm_is_busy_ = arm_is_busy;
}
void ControllerModule::setBaseIsBusy(bool base_is_busy)
{
    base_is_busy_ = base_is_busy;
}
void ControllerModule::setGripperIsBusy(bool gripper_is_busy)
{
    gripper_is_busy_ = gripper_is_busy;
}
bool ControllerModule::isArmBusy()
{
    return arm_is_busy_;
}
bool ControllerModule::isBaseBusy()
{
    return base_is_busy_;
}
bool ControllerModule::isGripperBusy()
{
    return gripper_is_busy_;
}
