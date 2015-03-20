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

#include "luh_youbot_manipulation/torque_controller/torque_controller.hpp"

using namespace luh_youbot_kinematics;

nAxesControllerTorque::nAxesControllerTorque(ros::NodeHandle &node):
    node_(&node)
{
    this->ta = 0.002;
    this->d_ta = 500;

    this->m_max.resize(5);
    this->m_max[0] = 9.5; // original Angabe 9.5 (9,5 Nm -> 1,818 A sind 32 % von Imax = 5,768 A)
    this->m_max[1] = 9.5; // original Angabe 9.5 (9,5 Nm -> 1,818 A sind 32 % von Imax = 5,768 A)
    this->m_max[2] = 6.0; // original Angabe 6.0 (6,0 Nm -> 1,791 A sind 46 % von Imax = 3,899 A) 4,5 Nm -> 34%
    this->m_max[3] = 5.0; // original Angabe 2.0 (5,0 Nm -> 1,381 A sind 50 % von Imax = 2,746 A)
    this->m_max[4] = 2.0; // original Angabe 1.0 (2,0 Nm -> 0,575 A sind 33 % von Imax = 1,75 A)

    this->k_p.resize(5);
    this->k_d.resize(5);
    this->k_i.resize(5);
    this->e_sum.resize(5);
    this->e_max.resize(5);
    this->e_max[0] = 0.7;
    this->e_max[1] = 0.6;
    this->e_max[2] = 0.5;
    this->e_max[3] = 0.2;
    this->e_max[4] = 0.1;

    this->k_feed = 0.0;

    this->k_f_n_1.resize(5);
    this->k_f_n_2.resize(5);
    this->k_f_n_3.resize(5);
    this->k_fr_n_1.resize(5);
    this->k_fr_n_2.resize(5);
    this->k_fr_n_3.resize(5);

    if(!loadParameters())
        ROS_ERROR("Failed to load torque controller parameters.");
}
nAxesControllerTorque::~nAxesControllerTorque() {

}

bool nAxesControllerTorque::loadParameters()
{
    if(!node_->getParam("torque_controller/k_p", k_p))
        return false;

    if(!node_->getParam("torque_controller/k_i", k_i))
        return false;

    if(!node_->getParam("torque_controller/k_d", k_d))
        return false;

    if(!node_->getParam("torque_controller/k_feed", k_feed))
        return false;

    if(!node_->getParam("torque_controller/k_f_n_1", k_f_n_1))
        return false;

    if(!node_->getParam("torque_controller/k_f_n_2", k_f_n_2))
        return false;

    if(!node_->getParam("torque_controller/k_f_n_3", k_f_n_3))
        return false;

    if(!node_->getParam("torque_controller/k_fr_n_1", k_fr_n_1))
        return false;

    if(!node_->getParam("torque_controller/k_fr_n_2", k_fr_n_2))
        return false;

    if(!node_->getParam("torque_controller/k_fr_n_3", k_fr_n_3))
        return false;

    return true;
}

void nAxesControllerTorque::reset()
{
    for(unsigned int i = 0; i < 5; i++)
    {
        e_sum[i] = 0.0;
    }
}

JointVector nAxesControllerTorque::getTorques(const JointPosition &position_command,
                                              const JointVelocity &velocity_command,
                                              const JointPosition &current_position,
                                              const JointVelocity &current_velocity,
                                              const JointVector   &joint_efforts)
{
    JointVector moment;

    for(unsigned int i = 0; i < 5; i++)
    {

        this->e_pos[i] = position_command[i] - current_position[i];
        this->e_vel[i] = velocity_command[i] - current_velocity[i];

        // I-Term
        e_sum[i] = e_pos[i] + e_sum[i];
        // Clipping to e_max
        if(fabs(e_sum[i]) > e_max[i])
        {
            if (e_sum[i] < 0)
                e_sum[i] = -1 * e_max[i];
            else
                e_sum[i] = 1 * e_max[i];
        }

        /* Torque Control */
        //        if(e_pos[i] > 0.2 || e_vel[i] > 0.3){
        if(e_pos[i] > 5.0 || e_vel[i] > 5.0)
        {
            ROS_WARN("error exceeded joint %d  p = %.2f  v = %.2f", i, e_pos[i], e_vel[i]);
            break; // todo: was dann?
        }
        else
        {
            m_joint_torques[i] = k_p[i] * e_pos[i] + k_i[i] * e_sum[i] * ta + k_d[i] * e_vel[i];


            /* Reibung */
            //            friction[i] = k_f_n_1[i] * tanh(k_f_n_2[i] * velocity_command[i]) + k_f_n_3[i] * velocity_command[i];

            /* Richtungsabhängige Reibung */
            if(velocity_command[i] > 0.0)
            {
                friction[i] = k_f_n_1[i] * tanh(k_f_n_2[i] * velocity_command[i]) + k_f_n_3[i] * velocity_command[i];
            }
            else
            {
                friction[i] = k_fr_n_1[i] * tanh(k_fr_n_2[i] * velocity_command[i]) + k_fr_n_3[i] * velocity_command[i];
            }
            /* Richtungsabhängige Reibung */

            reibvor[i] = k_feed * (joint_efforts[i] + friction[i]);
            moment[i] = m_joint_torques[i] + reibvor[i];

            //            moment[i] = m_joint_torques[i] + k_feed * m_joint_efforts[i] + friction[i];

            if(fabs(moment[i]) > m_max[i])
            {
                if (moment[i] < 0)
                    moment[i] = -1 * m_max[i];
                else
                    moment[i] = 1 * m_max[i];
            }
        }

    }

    return moment;
}

