/* *****************************************************************
 *
 * luh_youbot_kinematics
 *
 * Copyright (c) 2014,
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
 ******************************************************************/

/**
 * \file
 * @author Simon Aden (info@luhbots.de)
 * @date   November 2014
 *
 * @brief  Source file for all jointspace vectors.
 */

#include "luh_youbot_kinematics/arm_kinematics.h"

namespace luh_youbot_kinematics
{

//###################### CONSTRUCTORS ##################################################################################
JointVector::JointVector(): GenericVector(){}
JointPosition::JointPosition(): JointVector(){}
JointVelocity::JointVelocity(): JointVector(){}
JointAcceleration::JointAcceleration(): JointVector(){}

JointVector::JointVector(std::vector<double> val): GenericVector()
{
    this->setValues(val);
}
JointPosition::JointPosition(const std::vector<double> &val): JointVector()
{
    this->setValues(val);
}
JointVelocity::JointVelocity(const std::vector<double> &val): JointVector()
{
    this->setValues(val);
}
JointAcceleration::JointAcceleration(const std::vector<double> &val): JointVector()
{
    this->setValues(val);
}

//###################### TRANSFORMATIONS ###############################################################################
CylindricPosition JointPosition::toCylindric() const
{
    // (q1, q2, q3, q4, q5) -> (q1, r, z, theta, q5)

    CylindricPosition p;

    p.setQ1(this->q1());
    p.setTheta(this->q2() + this->q3() + this->q4());
    p.setR(L1 + L2 * sin(this->q2()) + L3 * sin(this->q2() + this->q3()) + L4 * sin(p.theta()));
    p.setZ(L0 + L2 * cos(this->q2()) + L3 * cos(this->q2() + this->q3()) + L4 * cos(p.theta()));
    p.setQ5(this->q5());

    return p;
}

void JointPosition::addOffset()
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] += JOINT_OFFSETS[i];
    }
}

void JointPosition::subtractOffset()
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] -= JOINT_OFFSETS[i];
    }
}

CylindricVelocity JointVelocity::toCylindric(const JointPosition &joint_position) const
{
    CylindricVelocity v;

    // accumulated joint angles
    double phi1 = joint_position[1];        // = q2
    double phi2 = phi1 + joint_position[2]; // = q2 + q3
    double phi3 = phi2 + joint_position[3]; // = q2 + q3 + q4

    // 3x3 Jacobian matrix elements
    double ja =  L2 * cos(phi1) + L3 * cos(phi2) + L4 * cos(phi3);
    double jb =  L3 * cos(phi2) + L4 * cos(phi3);
    double jc =  L4 * cos(phi3);
    double jd = -L2 * sin(phi1) - L3 * sin(phi2) - L4 * sin(phi3);
    double je = -L3 * sin(phi2) - L4 * sin(phi3);
    double jf = -L4 * sin(phi3);
    //    double jg =  1;
    //    double jh =  1;
    //    double ji =  1;
    //    -> optimized out

    // cylindrical velocity v = J * q
    v.setQ1(q1());
    v.setR(ja*q2() + jb*q3() + jc*q4());
    v.setZ(jd*q2() + je*q3() + jf*q4());
    v.setTheta(q2() + q3() + q4());
    v.setQ5(q5());

    return v;
}

CartesianPosition JointPosition::toCartesian() const
{
    CylindricPosition p = this->toCylindric();
    return p.toCartesian();
}

CartesianVelocity JointVelocity::toCartesian(const JointPosition &joint_position) const
{
    CylindricPosition p = joint_position.toCylindric();
    return this->toCylindric(joint_position).toCartesian(p);
}

// ########## IS REACHABLE #############################################################################################
bool JointPosition::isReachable()
{
    for(uint i=0; i<N_JOINTS; i++)
    {
        // numeric tolerance: 0.001
        if(MAX_JNT_POSITIONS[i] + 0.001 < (*this)[i] || MIN_JNT_POSITIONS[i] - 0.001 > (*this)[i])
            return false;
    }
    return true;
}

} // namespace luh_youbot_kinematics
