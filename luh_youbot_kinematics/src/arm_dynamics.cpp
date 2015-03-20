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

#include "luh_youbot_kinematics/arm_dynamics.h"

namespace luh_youbot_kinematics
{
//########## JOINT EFFORT CONSTRUCTOR ##################################################################################
JointEffort::JointEffort(): JointVector(){}
JointEffort::JointEffort(const std::vector<double> &val): JointVector()
{
    this->setValues(val);
}

//########## JOINT EFFORT CONVERSIONS ##################################################################################
CylindricEffort JointEffort::toCylindric(JointPosition joint_position)
{
    CylindricEffort f;

    // accumulated joint angles
    double phi1 = joint_position[1];        // = q2
    double phi2 = phi1 + joint_position[2]; // = q2 + q3
    double phi3 = phi2 + joint_position[3]; // = q2 + q3 + q4

    // 3x3 Jacobian matrix elements (transposed)
    double ja =  L2 * cos(phi1) + L3 * cos(phi2) + L4 * cos(phi3);
    double jd =  L3 * cos(phi2) + L4 * cos(phi3);
    double jg =  L4 * cos(phi3);
    double jb = -L2 * sin(phi1) - L3 * sin(phi2) - L4 * sin(phi3);
    double je = -L3 * sin(phi2) - L4 * sin(phi3);
    double jh = -L4 * sin(phi3);
    //    double jc =  1;
    //    double jf =  1;
    //    double ji =  1;
    // -> optimized out

    // determinant of transposed jacobian
    double det = ja*je + jb*jg + jd*jh - je*jg - jb*jd - ja*jh;

    if(det == 0.0)
    {
        f[0] = NAN;
        f[1] = NAN;
        f[2] = NAN;
        f[3] = NAN;
        f[4] = NAN;
    }
    else
    {
        // inverse transposed jacobian (= adjugate / determinant)
        double jai = (je    -    jh); // / det;
        double jbi = (   jh - jb   ); // / det;
        double jci = (jb    -    je); // / det;
        double jdi = (   jg - jd   ); // / det;
        double jei = (ja    -    jg); // / det;
        double jfi = (   jd - ja   ); // / det;
        double jgi = (jd*jh - je*jg); // / det;
        double jhi = (jb*jg - ja*jh); // / det;
        double jii = (ja*je - jb*jd); // / det;
        // -> division by determinant can be factored out and done in the next step

        // cylindric effort f = (J^T)^-1 * tau
        f[0] = q1();
        f[1] = (jai*q2() + jbi*q3() + jci*q4()) / det;
        f[2] = (jdi*q2() + jei*q3() + jfi*q4()) / det;
        f[3] = (jgi*q2() + jhi*q3() + jii*q4()) / det;
        f[4] = q5();
    }
    return f;
}

CartesianEffort JointEffort::toCartesian(JointPosition joint_position)
{
    return this->toCylindric(joint_position).toCartesian(joint_position);
}

//########## CYLINDRIC EFFORT CONSTRUCTOR ##############################################################################
CylindricEffort::CylindricEffort(): CylindricVector(){}
CylindricEffort::CylindricEffort(const std::vector<double> &val): CylindricVector()
{
    this->setValues(val);
}

//########## CYLINDRIC EFFORT CONVERSIONS ##############################################################################
JointEffort CylindricEffort::toJointspace(JointPosition joint_position)
{
    CylindricEffort tau;

    // accumulated joint angles
    double phi1 = joint_position[1];        // = q2
    double phi2 = phi1 + joint_position[2]; // = q2 + q3
    double phi3 = phi2 + joint_position[3]; // = q2 + q3 + q4

    // 3x3 Jacobian matrix elements (transposed)
    double ja =  L2 * cos(phi1) + L3 * cos(phi2) + L4 * cos(phi3);
    double jd =  L3 * cos(phi2) + L4 * cos(phi3);
    double jg =  L4 * cos(phi3);
    double jb = -L2 * sin(phi1) - L3 * sin(phi2) - L4 * sin(phi3);
    double je = -L3 * sin(phi2) - L4 * sin(phi3);
    double jh = -L4 * sin(phi3);
    //    double jc =  1;
    //    double jf =  1;
    //    double ji =  1;
    // -> optimized out


    // joint effort tau = (J^T) * f
    tau[0] = q1();
    tau[1] = ja*r() + jb*z() + theta();
    tau[2] = jd*r() + je*z() + theta();
    tau[3] = jg*r() + jh*z() + theta();
    tau[4] = q5();

    return tau;
}

CartesianEffort CylindricEffort::toCartesian(JointPosition joint_position)
{
    CartesianEffort f;

    double sq1 = sin(joint_position.q1());
    double cq1 = cos(joint_position.q1());

    CylindricPosition cyl_pos = joint_position.toCylindric();

    f.setX(this->r() * cq1 - sq1 * this->q1() / cyl_pos.r());
    f.setY(-this->r() * sq1 - cq1 * this->q1() / cyl_pos.r());
    f.setZ(this->z());
    f.setTheta(this->theta());
    f.setQ5(this->q5());

    return f;
}

//########## CARTESIAN EFFORT CONSTRUCTOR ##############################################################################
CartesianEffort::CartesianEffort(): CartesianVector(){}
CartesianEffort::CartesianEffort(const std::vector<double> &val): CartesianVector()
{
    this->setValues(val);
}

//########## CARTESIAN EFFORT CONVERSIONS ##############################################################################
CylindricEffort CartesianEffort::toCylindric(JointPosition joint_position)
{
    CylindricEffort f;

    double sq1 = sin(joint_position.q1());
    double cq1 = cos(joint_position.q1());

    CylindricPosition cyl_pos = joint_position.toCylindric();

    f.setR(this->x() * cq1 - this->y() * sq1);
    f.setQ1((-this->x() * sq1 - this->y() * cq1) * cyl_pos.r());
    f.setZ(this->z());
    f.setTheta(this->theta());
    f.setQ5(this->q5());
    return f;
}

JointEffort CartesianEffort::toJointspace(JointPosition joint_position)
{
    return this->toCylindric(joint_position).toJointspace(joint_position);
}

//########## YOUBOT ARM DYNAMICS CONSTRUCTOR ###########################################################################
YoubotArmDynamics::YoubotArmDynamics()
{
    // TODO: Initialise parameters
}

//########## GET JOINT EFFORT ##########################################################################################
JointEffort YoubotArmDynamics::getStaticJointEffort(JointPosition pos)
{
    // accumulated angles
    double phi1 = pos.q2();
    double phi2 = phi1 + pos.q3();
    double phi3 = phi2 + pos.q4();

    // center of mass positions relative to the corresponding joints (2,3,4,5)
    double r_s22 = stat_p_.com_radius_2 * sin(phi1 + stat_p_.com_angle_2);
    double r_s33 = stat_p_.com_radius_3 * sin(phi2 + stat_p_.com_angle_3);
    double r_s44 = stat_p_.com_radius_4 * sin(phi3 + stat_p_.com_angle_4);
    double r_s45 = stat_p_.com_radius_5* (sin(stat_p_.com_angle_5) * cos(pos.q5()) * cos(phi3)
                                          + cos(stat_p_.com_angle_5) * sin(phi3));

    // relative position of the next joint (or TCP in case of joint 4)
    double r_22 = L2 * sin(phi1);
    double r_33 = L3 * sin(phi2);

    // joint 5
    double trq_5 = stat_p_.com_radius_5 * sin(stat_p_.com_angle_5) * sin(phi3) * sin(pos.q5())
            * stat_p_.mass_5 * stat_p_.gravity;

    // joint 4
    double trq_4 = -r_s44 * stat_p_.mass_4 * stat_p_.gravity - r_s45 * stat_p_.mass_5 * stat_p_.gravity;

    // joint 3
    double trq_3 = trq_4 - r_33 * (stat_p_.mass_4 + stat_p_.mass_5) * stat_p_.gravity
            - r_s33 * stat_p_.mass_3 * stat_p_.gravity;

    // joint 2
    double trq_2 = trq_3 - r_22 * (stat_p_.mass_3 + stat_p_.mass_4 + stat_p_.mass_5) * stat_p_.gravity
            - r_s22 * stat_p_.mass_2 * stat_p_.gravity;

    // friction
    if(trq_2 > 0.01)
        trq_2 = std::max(trq_2 - stat_p_.friction_2, 0.0);
    else if(trq_2 < -0.01)
        trq_2 = std::min(trq_2 + stat_p_.friction_2, 0.0);

    if(trq_3 > 0.01)
        trq_3 = std::max(trq_3 - stat_p_.friction_3, 0.0);
    else if(trq_3 < -0.01)
        trq_3 = std::min(trq_3 + stat_p_.friction_3, 0.0);

    if(trq_4 > 0.01)
        trq_4 = std::max(trq_4 - stat_p_.friction_4, 0.0);
    else if(trq_4 < -0.01)
        trq_4 = std::min(trq_4 + stat_p_.friction_4, 0.0);

    if(trq_5 > 0.01)
        trq_5 = std::max(trq_5 - stat_p_.friction_5, 0.0);
    else if(trq_5 < -0.01)
        trq_5 = std::min(trq_5 + stat_p_.friction_5, 0.0);


    JointEffort effort;
    effort.setQ2(trq_2);
    effort.setQ3(trq_3);
    effort.setQ4(trq_4);
    effort.setQ5(trq_5);

    return effort;
}

 // TODO: fix dynamics calculations (inertia dependency on joint 5 is not correct yet)

//########## GET JOINT EFFORT ##########################################################################################
//JointEffort YoubotArmDynamics::getDynamicJointEffort()
//{
//    return getDynamicJointEffort(CylindricWrench());
//}

//########## GET JOINT EFFORT ##########################################################################################
//JointEffort YoubotArmDynamics::getDynamicJointEffort(CylindricWrench wrench)
//{
//    // accumulated joint angles
//    double phi1 = position_[1];        // = q2
//    double phi2 = phi1 + position_[2]; // = q2 + q3
//    double phi3 = phi2 + position_[3]; // = q2 + q3 + q4
//    double phi1_d = velocity_[1];
//    double phi2_d = phi1_d + velocity_[2];
//    double phi3_d = phi2_d + velocity_[3];
//    double phi1_dd = acceleration_[1];
//    double phi2_dd = phi1_d + acceleration_[2];
//    double phi3_dd = phi2_d + acceleration_[3];

//    // center of mass positions relative to joint 2
//    double r_s22 = p_.com_radius[1] * sin(phi1 + p_.com_angle[1]);
//    double r_s32 = L2 * sin(phi1) + p_.com_radius[2] * sin(phi2 + p_.com_angle[2]);
//    double r_s42 = L2 * sin(phi2) + L3 * sin(phi2) + p_.com_radius[3] * sin(phi3 + p_.com_angle[3]);

//    // center of mass accelerations
//    double r_s22_dd = p_.com_radius[1] *
//                      (
//                           phi1_dd       * cos(phi1 + p_.com_angle[1])
//                         - phi1_d*phi1_d * sin(phi1 + p_.com_angle[1])
//                      );
//    double r_s32_dd = L2 *
//                      (
//                           phi1_dd       * cos(phi1)
//                         - phi1_d*phi1_d * sin(phi1)
//                      )
//                      + p_.com_radius[2] *
//                        (
//                             phi2_dd       * cos(phi2 + p_.com_angle[2])
//                           - phi2_d*phi2_d * sin(phi2 + p_.com_angle[2])
//                        );
//    double r_s42_dd = L2 *
//                      (
//                           phi1_dd        * cos(phi1)
//                         - phi1_d*phi1_d * sin(phi1)
//                      )
//                      + L3 *
//                      (
//                           phi2_dd       * cos(phi2)
//                         - phi2_d*phi2_d * sin(phi2)
//                      )
//                      + p_.com_radius[3] *
//                      (
//                           phi3_dd       * cos(phi3 + p_.com_angle[3])
//                         - phi3_d*phi3_d * sin(phi3 + p_.com_angle[3])
//                      );

//    double z_s22_dd = -p_.com_radius[1] *
//                      (    phi1_dd       * sin(phi1 + p_.com_angle[1])
//                         + phi1_d*phi1_d * cos(phi1 + p_.com_angle[1])
//                      );
//    double z_s32_dd = -L2 *
//                      (
//                           phi1_dd       * sin(phi1)
//                         + phi1_d*phi1_d * cos(phi1)
//                      )
//                      - p_.com_radius[2] *
//                      (
//                           phi2_dd       * sin(phi2 + p_.com_angle[2])
//                         + phi2_d*phi2_d * cos(phi2 + p_.com_angle[2])
//                      );
//    double z_s42_dd = -L2 *
//                      (
//                           phi1_dd        * sin(phi1)
//                         + phi1_d*phi1_d * cos(phi1)
//                      )
//                      - L3 *
//                      (
//                           phi2_dd       * sin(phi2)
//                         + phi2_d*phi2_d * cos(phi2)
//                      )
//                      - p_.com_radius[3] *
//                      (
//                           phi3_dd       * sin(phi3 + p_.com_angle[3])
//                         + phi3_d*phi3_d * cos(phi3 + p_.com_angle[3])
//                      );

//    // angular velocities
//    double w_d_1 = velocity_[0];
//    double w_d_2 = phi1_d;
//    double w_d_3 = phi2_d;
//    double w_d_4 = phi3_d;
//    double w_d_5 = velocity_[5];

//    // center of mass positions relative to the corresponding joints (2,3,4)
//    double z_s22 = p_.com_radius[1] * cos(phi1 + p_.com_angle[1]);
//    double r_s33 = p_.com_radius[2] * sin(phi2 + p_.com_angle[2]);
//    double z_s33 = p_.com_radius[2] * cos(phi2 + p_.com_angle[2]);
//    double r_s44 = p_.com_radius[3] * sin(phi3 + p_.com_angle[3]);
//    double z_s44 = p_.com_radius[3] * cos(phi3 + p_.com_angle[3]);

//    // relative position of the next joint (or TCP in case of joint 4)
//    double r_22 = L2 * sin(phi1);
//    double z_22 = L2 * cos(phi1);
//    double r_33 = L3 * sin(phi2);
//    double z_33 = L3 * cos(phi2);
//    double r_44 = L4 * sin(phi3);
//    double z_44 = L4 * cos(phi3);


//    // joint 5
//    double trq_5 = p_.inertia_x[4] * w_d_5 - wrench.q5();

//    // joint 4
//    double inertia_45 = p_.inertia_x[3] + p_.inertia_y * cos(position_.q5()) + p_.inertia_z * sin(position_.q5()); //?
//    double fr_4 = p_.mass[3] * r_s42_dd - wrench.r();
//    double fz_4 = p_.mass[3] * z_s42_dd - wrench.z() + p_.mass[3] * p_.g;
//    double trq_4 = inertia_45 * w_d_4 - wrench.theta()
//                   - (z_44 - z_s44) * wrench.r()
//                   + (r_44 - r_s44) * wrench.z()
//                   - r_s44 * fz_4 - z_s44 * fr_4;

//    // joint 3
//    double fr_3 = p_.mass[2] * r_s32_dd + fr_4;
//    double fz_3 = p_.mass[2] * z_s32_dd + fz_4 + p_.mass[2] * p_.g;
//    double trq_3 = p_.inertia_x[2] * w_d_3 + trq_4
//                   + (z_33 - z_s33) * fr_4
//                   - (r_33 - r_s33) * fz_4
//                   - r_s33 * fz_3 - z_s33 * fr_3;

//    // joint 2
//    double fr_2 = p_.mass[1] * r_s22_dd + fr_3;
//    double fz_2 = p_.mass[1] * z_s22_dd + fz_3 + p_.mass[1] * p_.g;
//    double trq_2 = p_.inertia_x[1] * w_d_2 + trq_3
//                   + (z_22 - z_s22) * fr_3
//                   - (r_22 - r_s22) * fz_3
//                   - r_s22 * fz_2 - z_s22 * fr_2;

//    // joint 1
//    // TODO: link5 beachten bei p_.inertia_y[3] und p_.inertia_z[3]
//    double inertia = p_.inertia_x[0]
//                     + r_s22*r_s22*p_.mass[1] + r_s32*r_s32*p_.mass[2] + r_s42*r_s42*p_.mass[3]
//                     + sin(phi1) * p_.inertia_y[1] + sin(phi2) * p_.inertia_y[2] + sin(phi3) * p_.inertia_y[3]
//                     + cos(phi1) * p_.inertia_z[1] + cos(phi2) * p_.inertia_z[2] + cos(phi3) * p_.inertia_z[3]; // ?
//    double trq_1 = inertia * w_d_1 - wrench.q5();

//    JointEffort tau;
//    tau.setQ1(trq_1);
//    tau.setQ2(trq_2);
//    tau.setQ3(trq_3);
//    tau.setQ4(trq_4);
//    tau.setQ5(trq_5);

//    // friction
//    for(uint i=0; i<N_JOINTS; i++)
//    {
//        if(velocity_[i] > 0.0)
//            tau[i] += p_.friction_p_1[i] * tanh(p_.friction_p_1[i] * velocity_[i])
//                    + p_.friction_p_1[i] * velocity_[i];
//        else
//            tau[i] += p_.friction_n_1[i] * tanh(p_.friction_n_1[i] * velocity_[i])
//                    + p_.friction_n_1[i] * velocity_[i];
//    }

//    return tau;
//}

//########## GET JOINT EFFORT ##########################################################################################
//JointEffort YoubotArmDynamics::getDynamicJointEffort(CartesianWrench wrench)
//{
//    return getJointEffort(wrench.toCylindric(position_));
//}

}// namespace luh_youbot_kinematics
