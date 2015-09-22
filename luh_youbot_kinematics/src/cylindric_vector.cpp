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
 * @brief  Source file for all cylindrical vectors.
 */

#include "luh_youbot_kinematics/arm_kinematics.h"

namespace luh_youbot_kinematics
{

//###################### CONSTRUCTORS ##################################################################################
CylindricVector::CylindricVector(): GenericVector(){}
CylindricPosition::CylindricPosition(): CylindricVector(){}
CylindricAcceleration::CylindricAcceleration(): CylindricVector(){}
CylindricVelocity::CylindricVelocity(): CylindricVector(){}

CylindricVector::CylindricVector(std::vector<double> val): GenericVector()
{
    this->setValues(val);
}
CylindricPosition::CylindricPosition(std::vector<double> val): CylindricVector()
{
    this->setValues(val);
}
CylindricVelocity::CylindricVelocity(std::vector<double> val): CylindricVector()
{
    this->setValues(val);
}
CylindricAcceleration::CylindricAcceleration(std::vector<double> val): CylindricVector()
{
    this->setValues(val);
}

//###################### TRANSFORMATIONS ###############################################################################
JointPosition CylindricPosition::toJointspace(const JointPosition &reference_position, bool same_config) const
{
    if(same_config)
    {
        // the sign of joint 3 determines the elbow up or down configuration
        bool elbow_down = (reference_position.q3() < 0);

        return this->toJointspace(elbow_down);
    }
    else
    {
        JointPosition conf1 = this->toJointspace(true);
        JointPosition conf2 = this->toJointspace(false);

        double err1 = 0;
        double err2 = 0;
        double d;

        for(uint i=0; i<N_JOINTS; i++)
        {
            d = conf1[i] - reference_position[i];
            err1 += d*d;
            d = conf2[i] - reference_position[i];
            err2 += d*d;
        }

        if(err1 < err2  && conf1.isReachable() && conf1.isValid())
            return conf1;
        else return conf2;
    }
}

JointPosition CylindricPosition::toJointspace(bool elbow_down) const
{
    JointPosition q;

    // the sign in the calculation depends on the configuration
    double sign = -1.0;
    if(elbow_down)
        sign = 1.0;

    q.setQ1(this->q1());
    q.setQ5(this->q5());
    double rs = this->r() - L1 - L4 * sin(this->theta());
    double zs = this->z() - L0 - L4 * cos(this->theta());

    double zs_2 = zs*zs;
    double rs_2 = rs*rs;
    double l2_2 = L2*L2;
    double l3_2 = L3*L3;

    // check if position is valid
    double max_length = L2 + L3;
    if(rs_2 + zs_2 >= max_length * max_length)
    {
        q.setQ2(NAN);
        q.setQ3(NAN);
        q.setQ4(NAN);
        return q;
    }

    double arg1 = (l2_2 + l3_2 - rs_2 - zs_2) / (2 * L2 * L3);
    double arg2 = (l2_2 + rs_2 + zs_2 - l3_2) / (2 * sqrt(rs_2 + zs_2) * L2);

    // make sure that |arg| <= 1
    arg1 = std::min(1.0, std::max(-1.0, arg1));
    arg2 = std::min(1.0, std::max(-1.0, arg2));

    q.setQ3(sign *(acos(arg1) - M_PI));
    q.setQ2(atan2(rs, zs) + sign * acos(arg2));
    q.setQ4(theta() - q.q2() - q.q3());

    return q;
}

JointPosition CylindricPosition::toJointspace() const
{
    // if no configuration is given, the elbow configuration follows the endeffector orientation
    bool elbow_up = this->r() > 0 && this->theta() > M_PI/2 || this->r() < 0 && this->theta() > -M_PI/2;

    JointPosition jnt_pos = this->toJointspace(!elbow_up);

    // if the chosen elbow configuration is unreachable, the other one is returned
    if(jnt_pos.isReachable())
        return jnt_pos;
    else
        return this->toJointspace(elbow_up);
}

CartesianPosition CylindricPosition::toCartesian() const
{
    CartesianPosition p;
    p.setX(this->r() * cos(this->q1()) + ARM_LINK_0_X_OFFSET);
    p.setY(-this->r() * sin(this->q1()));
    p.setZ(this->z());
    p.setTheta(this->theta());
    p.setQ5(this->q5());

    return p;
}

JointVelocity CylindricVelocity::toJointspace(const JointPosition &joint_positions) const
{
    // joint velocity
    JointVelocity q;

    // accumulated joint angles
    double phi1 = joint_positions[1];        // = q2
    double phi2 = phi1 + joint_positions[2]; // = q2 + q3
    double phi3 = phi2 + joint_positions[3]; // = q2 + q3 + q4

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

    // determinant of jacobian
    double det = ja*je + jb*jf + jc*jd - jc*je - jb*jd - ja*jf;

    if(det == 0.0)
    {
        q[0] = NAN;
        q[1] = NAN;
        q[2] = NAN;
        q[3] = NAN;
        q[4] = NAN;
    }
    else
    {
        // inverse jacobian (= adjugate / determinant)
        double jai = (je    -    jf); // / det;
        double jbi = (jc    -    jb); // / det;
        double jci = (jb*jf - jc*je); // / det;
        double jdi = (jf    -    jd); // / det;
        double jei = (ja    -    jc); // / det;
        double jfi = (jc*jd - ja*jf); // / det;
        double jgi = (jd    -    je); // / det;
        double jhi = (jb    -    ja); // / det;
        double jii = (ja*je - jb*jd); // / det;
        // -> division by determinant can be factored out and done in the next step

        // joint velocities q = J^-1 * v
        q[0] = q1();
        q[1] = (jai*r() + jbi*z() + jci*theta()) / det;
        q[2] = (jdi*r() + jei*z() + jfi*theta()) / det;
        q[3] = (jgi*r() + jhi*z() + jii*theta()) / det;
        q[4] = q5();
    }

    return q;
}

CartesianVelocity CylindricVelocity::toCartesian(const CylindricPosition &position) const
{
    CartesianVelocity v;

    double sq1 = sin(position.q1());
    double cq1 = cos(position.q1());

    v.setX(this->r() * cq1 - position.r() * sq1 * this->q1());
    v.setY(-this->r() * sq1 - position.r() * cq1 * this->q1());
    v.setZ(this->z());
    v.setTheta(this->theta());
    v.setQ5(this->q5());

    return v;
}

CartesianAcceleration CylindricAcceleration::toCartesian(const CylindricVelocity &velocity,
                                                         const CylindricPosition &position)const
{
    CartesianAcceleration a;

    double sq1 = sin(position.q1());
    double cq1 = cos(position.q1());

    double f1 = this->r() - position.r() * velocity.q1() * velocity.q1();
    double f2 = -2 * velocity.r() * velocity.q1() + position.r() * this->q1();

    a.setX(f1 * cq1 + f2 * sq1);
    a.setY(f2 * cq1 - f1 * sq1);
    a.setZ(this->z());
    a.setTheta(this->theta());
    a.setQ5(this->q5());

    return a;
}

//###################### VERIFICATION ##################################################################################
bool CylindricPosition::isReachable()
{
    double rs = this->r() - L1 - L4 * sin(this->theta());
    double zs = this->z() - L0 - L4 * cos(this->theta());

    double max_length = L2 + L3;
    return (rs * rs + zs * zs) < (max_length * max_length);
}

} // namespace luh_youbot_kinematics
