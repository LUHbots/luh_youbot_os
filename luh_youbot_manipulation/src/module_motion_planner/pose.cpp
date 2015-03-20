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

#include "luh_youbot_manipulation/module_motion_planner/pose.h"
#include <cmath>

using namespace luh_youbot_kinematics;

Pose::Pose():is_goal(false), is_start(false), precursor(NULL), status(UNKNOWN)
{
}


double Pose::euclidean_distance(const Pose &p1, const Pose &p2)
{
    double dq1 = p1.joint_position.q1() - p2.joint_position.q1();
    double dq2 = p1.joint_position.q2() - p2.joint_position.q2();
    double dq3 = p1.joint_position.q3() - p2.joint_position.q3();
    double dq4 = p1.joint_position.q4() - p2.joint_position.q4();
    double dq5 = p1.joint_position.q5() - p2.joint_position.q5();

    return sqrt(dq1*dq1 + dq2*dq2 + dq3*dq3 + dq4*dq4 + dq5*dq5);
}

double Pose::weighted_euclidean_distance(const Pose &p1, const Pose &p2, const JointVector &weights)
{
    double dq1 = p1.joint_position.q1() - p2.joint_position.q1() * weights[0];
    double dq2 = p1.joint_position.q2() - p2.joint_position.q2() * weights[1];
    double dq3 = p1.joint_position.q3() - p2.joint_position.q3() * weights[2];
    double dq4 = p1.joint_position.q4() - p2.joint_position.q4() * weights[3];
    double dq5 = p1.joint_position.q5() - p2.joint_position.q5() * weights[4];

    return sqrt(dq1*dq1 + dq2*dq2 + dq3*dq3 + dq4*dq4 + dq5*dq5);
}

double Pose::chessboard_distance(const Pose &p1, const Pose &p2)
{
    double dq1 = std::fabs(p1.joint_position.q1() - p2.joint_position.q1());
    double dq2 = std::fabs(p1.joint_position.q2() - p2.joint_position.q2());
    double dq3 = std::fabs(p1.joint_position.q3() - p2.joint_position.q3());
    double dq4 = std::fabs(p1.joint_position.q4() - p2.joint_position.q4());
    double dq5 = std::fabs(p1.joint_position.q5() - p2.joint_position.q5());

    return std::max(dq1, std::max(dq2, std::max(dq3, std::max(dq4, dq5))));
}
double Pose::euclidean_distance(const JointPosition &p1, const Pose &p2)
{
    double dq1 = p1.q1() - p2.joint_position.q1();
    double dq2 = p1.q2() - p2.joint_position.q2();
    double dq3 = p1.q3() - p2.joint_position.q3();
    double dq4 = p1.q4() - p2.joint_position.q4();
    double dq5 = p1.q5() - p2.joint_position.q5();

    return sqrt(dq1*dq1 + dq2*dq2 + dq3*dq3 + dq4*dq4 + dq5*dq5);
}

double Pose::weighted_euclidean_distance(const JointPosition &p1, const Pose &p2, const JointVector &weights)
{
    double dq1 = p1.q1() - p2.joint_position.q1() * weights[0];
    double dq2 = p1.q2() - p2.joint_position.q2() * weights[1];
    double dq3 = p1.q3() - p2.joint_position.q3() * weights[2];
    double dq4 = p1.q4() - p2.joint_position.q4() * weights[3];
    double dq5 = p1.q5() - p2.joint_position.q5() * weights[4];

    return sqrt(dq1*dq1 + dq2*dq2 + dq3*dq3 + dq4*dq4 + dq5*dq5);
}

double Pose::chessboard_distance(const JointPosition &p1, const Pose &p2)
{
    double dq1 = std::fabs(p1.q1() - p2.joint_position.q1());
    double dq2 = std::fabs(p1.q2() - p2.joint_position.q2());
    double dq3 = std::fabs(p1.q3() - p2.joint_position.q3());
    double dq4 = std::fabs(p1.q4() - p2.joint_position.q4());
    double dq5 = std::fabs(p1.q5() - p2.joint_position.q5());

    return std::max(dq1, std::max(dq2, std::max(dq3, std::max(dq4, dq5))));
}

double Pose::duration(const Pose &p1, const Pose &p2,const JointVelocity &velocities)
{
    double dq1 = std::fabs(p1.joint_position.q1() - p2.joint_position.q1()) / velocities.q1();
    double dq2 = std::fabs(p1.joint_position.q2() - p2.joint_position.q2()) / velocities.q2();
    double dq3 = std::fabs(p1.joint_position.q3() - p2.joint_position.q3()) / velocities.q3();
    double dq4 = std::fabs(p1.joint_position.q4() - p2.joint_position.q4()) / velocities.q4();
    double dq5 = std::fabs(p1.joint_position.q5() - p2.joint_position.q5()) / velocities.q5();

    return std::max(dq1, std::max(dq2, std::max(dq3, std::max(dq4, dq5))));
}

double Pose::duration(const JointPosition &p1, const Pose &p2, const JointVelocity &velocities)
{
    double dq1 = std::fabs(p1.q1() - p2.joint_position.q1()) / velocities.q1();
    double dq2 = std::fabs(p1.q2() - p2.joint_position.q2()) / velocities.q2();
    double dq3 = std::fabs(p1.q3() - p2.joint_position.q3()) / velocities.q3();
    double dq4 = std::fabs(p1.q4() - p2.joint_position.q4()) / velocities.q4();
    double dq5 = std::fabs(p1.q5() - p2.joint_position.q5()) / velocities.q5();

    return std::max(dq1, std::max(dq2, std::max(dq3, std::max(dq4, dq5))));
}
