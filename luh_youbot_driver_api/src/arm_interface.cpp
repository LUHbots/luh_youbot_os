/* *****************************************************************
 *
 * luh_youbot_driver_api
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
 *
 *
 *
 * === Based on youbot_oodl with original license header: ===
 *
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Sebastian Blumenthal
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "luh_youbot_driver_api/arm_interface.h"

namespace ykin = luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
YoubotArmInterface::YoubotArmInterface(std::string name, YoubotConfiguration &config):
    name_(name),
    config_(&config),
    has_new_gripper_command_(false),
    has_new_arm_command_(false),
    arm_(NULL)
{

}

//########## DESTRUCTOR ################################################################################################
YoubotArmInterface::~YoubotArmInterface()
{
    delete arm_;
}

//########## INITIALISE ################################################################################################
void YoubotArmInterface::initialise(bool use_standard_gripper, bool use_luh_gripper_v3)
{
    // === PARAMETERS ===
    config_->node_handle->param("youbot_oodl_driver/disable_ramp", ramp_is_disabled_, false);
    config_->node_handle->param("youbot_oodl_driver/max_effort_max_duration", max_effort_max_duration_, 0.5);
    if(!ros::param::get("youbot_oodl_driver/max_effort", max_effort_))
    ROS_WARN("Failed to get max_effort parameter.");

   // === INITIALISATION ===
    arm_index_ = config_->num_arms++;
    std::stringstream ss;
    ss << "arm_" << arm_index_+1 << "/";
    topic_prefix_ = ss.str();
    mode_ = VELOCITY;
    gripper_finger_names_.resize(2);
    torque_command_.assign(5, 0.0);
    velocity_command_.assign(5, 0.0);
    position_command_.assign(5, 0.0);
    gripper_command_.assign(2, 0.0);

    effort_watchdog_time_.assign(ykin::N_JOINTS, ros::Time(0));

    use_standard_gripper_ = use_standard_gripper;
    use_luh_gripper_v3_=use_luh_gripper_v3;

    try
    {
        ROS_INFO("Initialising %s...", name_.c_str());
        ROS_INFO("Configuration file path: %s", config_->config_path.c_str());

        // === INITIALISE ARM ===
        arm_ = new youbot::YouBotManipulator(name_, config_->config_path);

        /* take joint names form configuration files */
        youbot::JointName joint_name_parameter;
        std::string joint_name;
        for (int i = 0; i < 5; ++i)
        {
            // get joint name
            arm_->getArmJoint(i + 1).getConfigurationParameter(joint_name_parameter);
            joint_name_parameter.getParameter(joint_name);
            joint_names_.push_back(joint_name);

            ROS_INFO("Joint %i for arm %s has name: %s", i + 1, name_.c_str(), joint_names_[i].c_str());
        }

        // commutation and calibration
        arm_->doJointCommutation();
        arm_->calibrateManipulator();


        // === INITIALISE GRIPPER ===
        if(use_standard_gripper_)
        {
            youbot::GripperBarName bar_name;
            std::string gripper_bar_name;

            arm_->getArmGripper().getGripperBar1().getConfigurationParameter(bar_name);
            bar_name.getParameter(gripper_bar_name);
            gripper_finger_names_[LEFT_FINGER_INDEX] = gripper_bar_name;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 1, name_.c_str(), gripper_bar_name.c_str());

            arm_->getArmGripper().getGripperBar2().getConfigurationParameter(bar_name);
            bar_name.getParameter(gripper_bar_name);
            gripper_finger_names_[RIGHT_FINGER_INDEX] = gripper_bar_name;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 2, name_.c_str(), gripper_bar_name.c_str());

            arm_->calibrateGripper();
        }
        if(use_luh_gripper_v3_)
        {
            int slaveNumber=9; //Erstmal hardgecoded TODO
            std::string gripper_bar_name;
            gripper_bar_name="gripper_finger_joint_l";
            gripper_finger_names_[LEFT_FINGER_INDEX] = gripper_bar_name;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 1, name_.c_str(), gripper_bar_name.c_str());
            gripper_bar_name="gripper_finger_joint_r";
            gripper_finger_names_[RIGHT_FINGER_INDEX] = gripper_bar_name;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 2, name_.c_str(), gripper_bar_name.c_str());


            luh_gripper_v3_=new arduino_gripper();
            luh_gripper_v3_->initAdruinoGripper(slaveNumber,config_->config_path); //TODO Slave Number
            luh_gripper_v3_read_state_counter_=0;
            ROS_INFO("Kalibrating Gripper...");
            sleep(3);
            luh_gripper_v3_->setPosition(0.03);
            sleep(3);
            luh_gripper_v3_->setPosition(0.06);
            sleep(3);
            ROS_INFO("Gripper kalibrated.");





            int gripper_max_effort=100;

            if(ros::param::get("luh_youbot_controller/gripper_max_effort", gripper_max_effort))
            {
                ROS_INFO("Successfull got gripper max_effort");
            }

            ROS_INFO("Setting the gripper efford threshhold to %i",gripper_max_effort);
            luh_gripper_v3_->setEffort(gripper_max_effort);


        }
    }
    catch (std::exception& e)
    {
        delete arm_;
        arm_ = NULL;
        delete luh_gripper_v3_;
        luh_gripper_v3_= NULL;
        std::string errorMessage = e.what();
        ROS_FATAL("Cannot open youBot driver: \n %s ", errorMessage.c_str());
        ROS_ERROR("Arm \"%s\" could not be initialized.", name_.c_str());
        ROS_INFO("System has %i initialized arm(s).", --config_->num_arms);
        return;
    }

    // === ROS COMMUNICATION ===
    std::string topic_name = topic_prefix_;
    topic_name.append("joint_states");
    joint_state_publisher_ = config_->node_handle->advertise<sensor_msgs::JointState>(topic_name, 1);


    /* setup services*/
    std::string service_name = topic_prefix_;
    service_name.append("switchOffMotors");
    switch_off_server_ = config_->node_handle->advertiseService(service_name,
                                                                &YoubotArmInterface::switchOffArmMotorsCallback, this);

    service_name = topic_prefix_;
    service_name.append("switchOnMotors");
    switch_on_server_ = config_->node_handle->advertiseService(service_name,
                                                               &YoubotArmInterface::switchOnArmMotorsCallback, this);

    service_name = topic_prefix_;
    service_name.append("calibrate");
    calibrate_server_ = config_->node_handle->advertiseService(service_name,
                                                               &YoubotArmInterface::calibrateArmCallback, this);


    // === INIT JOINT STATE MESSAGE ===
    for(uint i=0; i<joint_names_.size(); i++)
    {
        joint_state_.name.push_back(joint_names_[i]);
    }

    if(use_standard_gripper_)
    {
        joint_state_.name.push_back(gripper_finger_names_[LEFT_FINGER_INDEX]);
        joint_state_.name.push_back(gripper_finger_names_[RIGHT_FINGER_INDEX]);

        joint_state_.position.resize(7);
        joint_state_.velocity.resize(7);
        joint_state_.effort.resize(7);
    }
    else if(use_luh_gripper_v3_)
    {
        joint_state_.name.push_back(gripper_finger_names_[LEFT_FINGER_INDEX]);
        joint_state_.name.push_back(gripper_finger_names_[RIGHT_FINGER_INDEX]);

        joint_state_.position.resize(7);
        joint_state_.velocity.resize(7);
        joint_state_.effort.resize(7);

    }
    else
    {
        joint_state_.position.resize(5);
        joint_state_.velocity.resize(5);
        joint_state_.effort.resize(5);
    }
    gripper_position_=0;
    gripper_velocity_=0;
    gripper_effort_=0;

    // === MOVE TO VALID INITIAL POSE ===

    readState();

    ykin::JointPosition init_pose;

    for(uint i=0; i<ykin::N_JOINTS; i++)
    {
        if(joint_position_[i] > ykin::MAX_JNT_POSITIONS[i] - 0.01)
            init_pose[i] = ykin::MAX_JNT_POSITIONS[i] - 0.01;

        else if(joint_position_[i] < ykin::MIN_JNT_POSITIONS[i] + 0.01)
            init_pose[i] = ykin::MIN_JNT_POSITIONS[i] + 0.01;

        else
            init_pose[i] = joint_position_[i];
    }

    ROS_INFO("Moving arm away from joint limits...");
    setJointPositions(init_pose);

    // enable ramp while moving to home pose
    bool ramp_is_disabled_old = ramp_is_disabled_;
    ramp_is_disabled_ = false;
    for (int i = 0; i < 5; ++i)
    {
        // disable/enable ramp generator
        youbot::RampGeneratorSpeedAndPositionControl use_ramp;
        use_ramp.setParameter(true);
        arm_->getArmJoint(i + 1).setConfigurationParameter(use_ramp);
    }
    writeCommands();
    ros::Duration(1.0).sleep();
    ramp_is_disabled_ = ramp_is_disabled_old;

    // === SET RAMP GENERATOR PARAMETER ===
    for (int i = 0; i < 5; ++i)
    {
        // disable/enable ramp generator
        youbot::RampGeneratorSpeedAndPositionControl use_ramp;
        use_ramp.setParameter(!ramp_is_disabled_);
        arm_->getArmJoint(i + 1).setConfigurationParameter(use_ramp);
    }

    ROS_INFO("Arm \"%s\" is initialized.", name_.c_str());
    ROS_INFO("System has %i initialized arm(s).", config_->num_arms);

}

//########## SECURITY CHECK ############################################################################################
int YoubotArmInterface::securityCheck()
{
    ros::Time time_now = ros::Time::now();
    for(uint i=0; i<ykin::N_JOINTS; i++)
    {
        if(fabs(joint_torque_[i]) < max_effort_[i] || effort_watchdog_time_[i] == ros::Time(0))
        {
            effort_watchdog_time_[i] = time_now;
        }
        else if((time_now - effort_watchdog_time_[i]).toSec() > max_effort_max_duration_)
        {
                return i+1;
        }
    }
    return 0;
}

//########## READ STATE ################################################################################################
void YoubotArmInterface::readState()
{
    if(arm_ == NULL)
        return;

    last_sample_time_ = current_sample_time_;
    current_sample_time_ = ros::Time::now();
    delta_t_ = (current_sample_time_ - last_sample_time_).toSec();

    youbot::JointSensedAngle current_angle;
    youbot::JointSensedVelocity current_velocity;
    youbot::JointSensedTorque current_torque;
//    youbot::JointSensedCurrent current_current;

    // ensure that all joint values will be sent at the same time
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);

    /* fill joint state message */
    joint_state_.header.stamp = current_sample_time_;

    for (int i = 0; i < 5; ++i)
    {
        arm_->getArmJoint(i + 1).getData(current_angle);
        arm_->getArmJoint(i + 1).getData(current_velocity);
        arm_->getArmJoint(i + 1).getData(current_torque);

        joint_state_.position[i] = current_angle.angle.value();
        joint_state_.velocity[i] = current_velocity.angularVelocity.value();
        joint_state_.effort[i] = current_torque.torque.value();
    }

    // ensure that all joint values will be sent at the same time
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

    luh_gripper_v3_read_state_counter_++;

    if(use_luh_gripper_v3_ && luh_gripper_v3_read_state_counter_>20)
    {
        luh_gripper_v3_read_state_counter_=0;

        std::vector<bool> JointReachedPosition(5);
        unsigned int StatusFlags;
        for (int i=0;i<5;i++)
        {

            arm_->getArmJoint(i+1).getStatus(StatusFlags);
            if(StatusFlags & POSITION_REACHED)
            {
                JointReachedPosition[i]=true;
            }else{
                JointReachedPosition[i]=false;
            }
        }
//        ROS_INFO("Status Flag PositioReached is: %i",StatusFlags & POSITION_REACHED);
//        ROS_INFO("Status Flag POSITION_MODE is: %i",StatusFlags & POSITION_MODE);
//        ROS_INFO("Status Flag TORQUE_MODE is: %i",StatusFlags & TORQUE_MODE);
//        ROS_INFO("Status Flag VELOCITY_MODE is: %i",StatusFlags & VELOCITY_MODE);

        if(JointReachedPosition[0] && JointReachedPosition[1] && JointReachedPosition[2] && JointReachedPosition[3] && JointReachedPosition[4])
        {
            float currentPosition=0.0;
            float currentVelocity=0.0;
            float currentEffort=0.0;
            luh_gripper_v3_->getPosition(currentPosition);
//            ROS_INFO("Gripper_Position= %f",currentPosition);
            luh_gripper_v3_->getVelocity(currentVelocity);
//            ROS_INFO("Gripper_Velocity= %f",currentVelocity);
            luh_gripper_v3_->getEffort(currentEffort);
//            ROS_INFO("Gripper_Effort= %f",currentEffort);
            joint_state_.position[5] = currentPosition/2;
            joint_state_.position[6] = currentPosition/2;
            joint_state_.velocity[5] = currentVelocity/2;
            joint_state_.velocity[6] = currentVelocity/2;
            joint_state_.effort[5]= currentEffort;
            joint_state_.effort[6]= currentEffort;
            gripper_position_=currentPosition;
            gripper_velocity_=currentVelocity;
            gripper_effort_=currentEffort;
        }

    }

    joint_position_.setValues(joint_state_.position);
    joint_position_.addOffset();
    joint_velocity_.setValues(joint_state_.velocity);
    joint_torque_.setValues(joint_state_.effort);
}

//########## WRITE COMMANDS ############################################################################################
bool YoubotArmInterface::writeCommands()
{
    if(arm_ == NULL)
        return false;

    // === SECURITY CHECK ===
    if(ramp_is_disabled_ && mode_ == POSITION)
    {
        double max_angle_diff = 0;
        int max_index = 0;
        for (int i = 0; i < static_cast<int>(position_command_.size()); ++i)
        {
            double diff = std::fabs(joint_state_.position[i] - position_command_[i]);
            if(diff > max_angle_diff)
            {
                max_angle_diff = diff;
                max_index = i;
            }
        }

        if(max_angle_diff > 0.12)
        {
            ROS_WARN("Position steps greater than 0.12 are not allowed if ramp is disabled.");
            ROS_WARN("Current position step is %f in joint %d", max_angle_diff, max_index+1);
            return false;
        }
    }

    // ensure that all joint values will be sent at the same time
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);

    bool has_succeeded = true;

    if(has_new_arm_command_)
    {
        if(mode_ == POSITION)
        {
            // === SET POSITION COMMAND ===
            youbot::JointAngleSetpoint desired_angle;

            for (int i = 0; i < 5; ++i)
            {
                desired_angle.angle = position_command_[i] * radian;
                try
                {
                    arm_->getArmJoint(i + 1).setData(desired_angle);
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: \n %s", i + 1, errorMessage.c_str());
                    has_succeeded = false;
                }
            }

        }
        else if(mode_ == VELOCITY)
        {
            // === SET VELOCITY COMMAND ===
            youbot::JointVelocitySetpoint desired_velocity;

            for (int i = 0; i < 5; ++i)
            {
                desired_velocity.angularVelocity = velocity_command_[i] * radian_per_second;
                try
                {
                    arm_->getArmJoint(i + 1).setData(desired_velocity);
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: \n %s", i + 1, errorMessage.c_str());
                    has_succeeded = false;
                }
            }
        }
        else
        {
            // === SET TORQUE COMMAND ===
            youbot::JointTorqueSetpoint desired_torque;

            for (int i = 0; i < 5; ++i)
            {
                desired_torque.torque = torque_command_[i] * newton_meter;
                try
                {
                    arm_->getArmJoint(i + 1).setData(desired_torque);
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: \n %s", i + 1, errorMessage.c_str());
                    has_succeeded = false;
                }
            }
        }
        // ensure that all joint values will be sent at the same time
        youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

        has_new_arm_command_ = false;
    }

    // === SET GRIPPER COMMAND ===
    if(has_new_gripper_command_ && use_standard_gripper_)
    {
        // ensure that all joint values will be send at the same time
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);

        youbot::GripperBarPositionSetPoint leftGripperFingerPosition;
        youbot::GripperBarPositionSetPoint rightGripperFingerPosition;

        rightGripperFingerPosition.barPosition = gripper_command_[RIGHT_FINGER_INDEX] * meter;
        leftGripperFingerPosition.barPosition = gripper_command_[LEFT_FINGER_INDEX] * meter;

        try
        {
            arm_->getArmGripper().getGripperBar1().setData(leftGripperFingerPosition);
            arm_->getArmGripper().getGripperBar2().setData(rightGripperFingerPosition);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot set the right gripper finger: \n %s", errorMessage.c_str());
            has_succeeded = false;
        }

        // ensure that all joint values will be send at the same time
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

        has_new_gripper_command_ = false;

        // set command as jointstat since gripper doesn't have sensors
        joint_state_.position[5] = gripper_command_[LEFT_FINGER_INDEX];
        joint_state_.position[6] = gripper_command_[RIGHT_FINGER_INDEX];
    }
    if(has_new_gripper_command_ && use_luh_gripper_v3_)
    {
        try{
            luh_gripper_v3_->setPosition(luh_gripper_v3_position_command_);
        }catch(std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot set the right gripper finger: \n %s", errorMessage.c_str());
            has_succeeded = false;
        }
        has_new_gripper_command_ = false;

        // set command as jointstat since gripper doesn't have sensors
//        joint_state_.position[5] = luh_gripper_v3_position_command_/2;
//        joint_state_.position[6] = luh_gripper_v3_position_command_/2;

    }

    return has_succeeded;
}

//########## STOP ######################################################################################################
void YoubotArmInterface::stop()
{
    if(arm_ == NULL)
        return;

    // enable ramp generator for safety reasons
    if(ramp_is_disabled_)
    {
        for (int i = 0; i < 5; ++i)
        {
            youbot::RampGeneratorSpeedAndPositionControl use_ramp;
            use_ramp.setParameter(true);
            arm_->getArmJoint(i + 1).setConfigurationParameter(use_ramp);
        }
        std::cout << "Ramp for arm " << arm_index_ +1 << " is enabled again." << std::endl;
    }

    delete arm_;
    arm_ = NULL;

    joint_state_publisher_.shutdown();

    calibrate_server_.shutdown();
    switch_off_server_.shutdown();
    switch_on_server_.shutdown();
}

//########## SET POSITIONS #############################################################################################
void YoubotArmInterface::setJointPositions(ykin::JointPosition positions)
{
    positions.subtractOffset();
    mode_ = POSITION;
    position_command_ = positions;
    has_new_arm_command_ = true;
}

//########## SET VELOCITIES ############################################################################################
void YoubotArmInterface::setJointVelocities(ykin::JointVelocity velocities)
{
    mode_ = VELOCITY;
    velocity_command_ = velocities;
    has_new_arm_command_ = true;
}

//########## SET TORQUES ###############################################################################################
void YoubotArmInterface::setJointTorques(ykin::JointVector torques)
{
    mode_ = TORQUE;
    torque_command_ = torques;
    has_new_arm_command_ = true;
}

//########## SET GRIPPER POSITION ######################################################################################
void YoubotArmInterface::setGripperPosition(double left, double right)
{
    if(use_standard_gripper_)
    {
        gripper_command_[LEFT_FINGER_INDEX] = left;
        gripper_command_[RIGHT_FINGER_INDEX] = right;

        has_new_gripper_command_ = true;
    }
    if(use_luh_gripper_v3_)
    {
        luh_gripper_v3_position_command_=fabs(left)+fabs(right);
        has_new_gripper_command_ = true;
    }
}

//########## SET GRIPPER WIDTH #########################################################################################
void YoubotArmInterface::setGripperPosition(double width)
{
    if(use_standard_gripper_)
    {
        gripper_command_[LEFT_FINGER_INDEX] = width / 2;
        gripper_command_[RIGHT_FINGER_INDEX] = width / 2;

        has_new_gripper_command_ = true;
    }

    if(use_luh_gripper_v3_)
    {
        luh_gripper_v3_position_command_=width;
        has_new_gripper_command_ = true;
    }
}

//########## SWITCH OFF MOTORS CALLBACK ################################################################################
bool YoubotArmInterface::switchOffArmMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    mode_ = TORQUE;
    for(uint i=0; i<torque_command_.size(); i++)
    {
        torque_command_[i] = 0;
    }

    has_new_arm_command_ = true;

    return true;
}

//########## SWITCH ON MOTORS CALLBACK #################################################################################
bool YoubotArmInterface::switchOnArmMotorsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    mode_ = VELOCITY;
    for(uint i=0; i<velocity_command_.size(); i++)
    {
        velocity_command_[i] = 0;
    }

    has_new_arm_command_ = true;

    return true;
}

//########## CALIBRATE ARM CALLBACK ####################################################################################
bool YoubotArmInterface::calibrateArmCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Calibrate the arm%i", arm_index_ + 1);

    if (arm_ != NULL)
    {
        try
        {
            arm_->calibrateManipulator(true);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot calibrate the arm: \n %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Arm%i not initialized!", arm_index_ + 1);
        return false;
    }
    return true;
}

//########## PUBLISH MESSAGES ##########################################################################################
void YoubotArmInterface::publishMessages()
{
    joint_state_publisher_.publish(joint_state_);
}

//########## GET JOINT POSITIION #######################################################################################
ykin::JointPosition YoubotArmInterface::getJointPosition()
{
    return joint_position_;
}

//########## GET JOINT VELOCITY ########################################################################################
ykin::JointVelocity YoubotArmInterface::getJointVelocity()
{
    return joint_velocity_;
}

//########## GET JOINT TORQUE ##########################################################################################
ykin::JointVector YoubotArmInterface::getJointTorque()
{
    return joint_torque_;
}

//########## ENABLE RAMP GENERATOR #####################################################################################
void YoubotArmInterface::enableRampGenerator(bool enable)
{
    for (int i = 0; i < 5; ++i)
    {
        youbot::RampGeneratorSpeedAndPositionControl use_ramp;
        use_ramp.setParameter(enable);
        arm_->getArmJoint(i + 1).setConfigurationParameter(use_ramp);
    }

    ramp_is_disabled_ = enable;
}
