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

#ifndef LUH_YOUBOT_MANIPULATION_N_AXES_CONTROLLER_TORQUE_HPP
#define LUH_YOUBOT_MANIPULATION_N_AXES_CONTROLLER_TORQUE_HPP

#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "luh_youbot_kinematics/arm_kinematics.h"

class nAxesControllerTorque {

public:

    nAxesControllerTorque(ros::NodeHandle &node);

    ~nAxesControllerTorque();

    luh_youbot_kinematics::JointVector getTorques(const luh_youbot_kinematics::JointPosition &position_command,
                                                  const luh_youbot_kinematics::JointVelocity &velocity_command,
                                                  const luh_youbot_kinematics::JointPosition &current_position,
                                                  const luh_youbot_kinematics::JointVelocity &current_velocity,
                                                  const luh_youbot_kinematics::JointVector &joint_efforts
                                                  = luh_youbot_kinematics::JointVector());
    void reset();
    bool loadParameters();

private:

    ros::NodeHandle* node_;

    luh_youbot_kinematics::JointVector m_joint_torques;

    luh_youbot_kinematics::JointPosition e_pos;
    luh_youbot_kinematics::JointPosition e_pos_alt;
    luh_youbot_kinematics::JointVelocity e_vel;

    std::vector <double> k_p;
    std::vector <double> k_d;
    std::vector <double> k_i;
    std::vector <double> m_max;

    std::vector <double> e_sum;
    std::vector <double> e_max;

    double ta;
    double d_ta;

    luh_youbot_kinematics::JointVector friction;
    std::vector <double> k_f_n_1;
    std::vector <double> k_f_n_2;
    std::vector <double> k_f_n_3;
    std::vector <double> k_fr_n_1;
    std::vector <double> k_fr_n_2;
    std::vector <double> k_fr_n_3;

    double k_feed;

    double time_begin;
    double time_passed;

    luh_youbot_kinematics::JointVector reibvor;

};

#endif // N_AXES_CONTROLLER_TORQUE_HPP
