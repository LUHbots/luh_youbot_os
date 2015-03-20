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
 * Author: Nikolaj Ponomarjow
 * Modified by: Simon Aden (simon.aden@mailbox.org)
 ******************************************************************/


#ifndef LUH_YOUBOT_MANIPULATION_YOUBOT_DYNAMICS_HPP
#define LUH_YOUBOT_MANIPULATION_YOUBOT_DYNAMICS_HPP

#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>

#include <Eigen/Dense>

#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "luh_youbot_kinematics/arm_kinematics.h"

class YoubotDynamics {

public:

    YoubotDynamics();

    ~YoubotDynamics();

    bool start();

    luh_youbot_kinematics::JointVector getEffort(luh_youbot_kinematics::JointPosition joint_position,
                                                 luh_youbot_kinematics::JointVelocity joint_velocity,
                                                 luh_youbot_kinematics::JointAcceleration joint_acceleration);

    void stop();

private:

    bool init();

    ros::NodeHandle* n;

    std::string prop_urdf_model;

    KDL::JntArray m_q;
    KDL::JntArray m_qdot;
    KDL::JntArray m_qdotdot;
    KDL::JntArray m_torques;

//    geometry_msgs::Wrench m_wrench_msg;
    KDL::Wrenches m_wrenches;
    KDL::Chain m_chain;

    KDL::ChainIdSolver_RNE *m_id_solver;

};

#endif // YOUBOT_DYNAMICS_HPP
