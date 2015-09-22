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

#ifndef LUH_YOUBOT_CONTROLLER_POSE_H
#define LUH_YOUBOT_CONTROLLER_POSE_H

#include <vector>
#include <string>
#include "luh_youbot_kinematics/arm_kinematics.h"

enum PoseStatus
{
    UNKNOWN,
    OPEN,
    CLOSED,
    BLOCKED
};

class Pose
{
public:
    Pose();

    std::string name;

    luh_youbot_kinematics::JointPosition joint_position;

    std::vector<Pose*> neighbors;
    std::vector<double> costs;

    double cost; /// cost from start to this pose
    double estimate; /// estimated cost from this pose to goal


    bool is_goal;
    bool is_start;
    Pose* precursor;
    PoseStatus status;

    static double euclidean_distance(const Pose &p1, const Pose &p2);
    static double chessboard_distance(const Pose &p1, const Pose &p2);
    static double euclidean_distance(const luh_youbot_kinematics::JointPosition &p1, const Pose &p2);
    static double chessboard_distance(const luh_youbot_kinematics::JointPosition &p1, const Pose &p2);
    static double duration(const luh_youbot_kinematics::JointPosition &p1, const Pose &p2,
                           const luh_youbot_kinematics::JointVelocity &velocities);
    static double duration(const Pose &p1, const Pose &p2, const luh_youbot_kinematics::JointVelocity &velocities);
    static double weighted_euclidean_distance(const Pose &p1, const Pose &p2, const luh_youbot_kinematics::JointVector &weights);
    static double weighted_euclidean_distance(const luh_youbot_kinematics::JointPosition &p1, const Pose &p2,
                                              const luh_youbot_kinematics::JointVector &weights);

};

#endif // POSE_H
