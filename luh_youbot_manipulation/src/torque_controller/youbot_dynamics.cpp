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

#include "luh_youbot_manipulation/torque_controller/youbot_dynamics.hpp"

using namespace luh_youbot_kinematics;

YoubotDynamics::YoubotDynamics()
{

    this->m_q.resize(5);
    this->m_qdot.resize(5);
    this->m_qdotdot.resize(5);
    this->m_torques.resize(5);

    this->init();

}
YoubotDynamics::~YoubotDynamics()
{

}

bool YoubotDynamics::init()
{

    //    this->n.getParam("robot_description", prop_urdf_model);
    //    KDL::Tree my_tree;
    //    if (!kdl_parser::treeFromString(prop_urdf_model, my_tree)) {
    //        ROS_ERROR("Failed to construct kdl tree");
    //        return false;
    //    }

    // Get path from ROS-Package
    prop_urdf_model = ros::package::getPath("luh_youbot_manipulation");
    prop_urdf_model.append("/resources/torque_controller/youbot_ros_angepasst.urdf");

    KDL::Tree my_tree;
    if (!kdl_parser::treeFromFile(prop_urdf_model, my_tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    if (!my_tree.getChain("arm_link_0", "gripper_tip", this->m_chain)) {
        /* "gripper_palm_link" Ursprung des Grippers */
        ROS_ERROR("Failed to construct subchain from urdf model");
        return false;
    }
    if (this->m_chain.getNrOfJoints() != 5) {
        ROS_ERROR("Nr of joints in urdf model is not correct: %d", this->m_chain.getNrOfJoints());
        return false;
    }

    m_id_solver = new KDL::ChainIdSolver_RNE(this->m_chain, KDL::Vector(0.0, 0.0, -9.81));

    SetToZero(m_q);
    SetToZero(m_qdot);
    SetToZero(m_qdotdot);
    SetToZero(m_torques);
    m_wrenches.resize(this->m_chain.getNrOfSegments());
    SetToZero(m_wrenches[m_wrenches.size() - 1]);

    return true;
}

JointVector YoubotDynamics::getEffort(JointPosition joint_position,
                                      JointVelocity joint_velocity,
                                      JointAcceleration joint_acceleration)
{
    JointVector efforts;
    joint_position.subtractOffset();
    for(uint i=0; i<N_JOINTS; i++)
    {
        m_q(i) = joint_position[i];
        m_qdot(i) = joint_velocity[i];
        m_qdotdot(i) = joint_acceleration[i];
    }

    //todo: wrenches?

    int ret = m_id_solver->CartToJnt(m_q, m_qdot, m_qdotdot, m_wrenches, m_torques);

    if (ret < 0)
    {
        ROS_ERROR("Could not calculate inverse dynamics: %d", ret);
    }
    else
    {
        for(unsigned int i = 0; i < m_torques.rows(); i++)
        {
            efforts[i] = m_torques(i);
        }
    }

    return efforts;
}
