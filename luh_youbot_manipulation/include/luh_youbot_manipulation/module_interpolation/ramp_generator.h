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

#ifndef LUH_YOUBOT_MANIPULATION_RAMP_GENERATOR_H
#define LUH_YOUBOT_MANIPULATION_RAMP_GENERATOR_H

#include "luh_youbot_kinematics/arm_kinematics.h"
#include "ros/time.h"

class RampGenerator
{
public:
    RampGenerator();

    void update();
    void setTargetPosition(luh_youbot_kinematics::GenericVector position, bool position_is_absolute);
    void setTargetVelocity(luh_youbot_kinematics::GenericVector velocity);
    luh_youbot_kinematics::GenericVector getVelocity();
    luh_youbot_kinematics::GenericVector getPosition();
    void setMaxVelocity(const luh_youbot_kinematics::GenericVector &max_vel);
    void setMaxAcceleration(const luh_youbot_kinematics::GenericVector &max_acc);
    void resetCurrentState(const luh_youbot_kinematics::GenericVector &velocity,
                           const luh_youbot_kinematics::GenericVector &position);
    bool targetPositionReached();

    void adjustVelocity(double factor);

protected:
    enum Phase
    {
        ACCELERATE,
        HOLD_VELOCITY,
        DECELERATE
    }state_;

    struct State
    {
        double a, v, s;
        luh_youbot_kinematics::GenericVector position, velocity;
    };

    State current_, last_, next_;

    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Publisher pub4;
    ros::Publisher pub5;

    luh_youbot_kinematics::GenericVector max_acceleration_;
    luh_youbot_kinematics::GenericVector max_velocity_;

    luh_youbot_kinematics::GenericVector initial_position_;
    luh_youbot_kinematics::GenericVector initial_velocity_;
    luh_youbot_kinematics::GenericVector goal_position_;
    luh_youbot_kinematics::GenericVector goal_velocity_;

    double a_max_, v_max_, s_max_;

    luh_youbot_kinematics::GenericVector vector_;

//    luh_youbot_kinematics::GenericVector current_position_;
//    luh_youbot_kinematics::GenericVector current_velocity_;
//    luh_youbot_kinematics::GenericVector current_acceleration_;

//    luh_youbot_kinematics::GenericVector position_command_;
//    luh_youbot_kinematics::GenericVector velocity_command_;

    bool velocity_mode_;
    bool got_target_;
    bool target_position_reached_;

    double start_time_;
    double current_time_;
    double delta_t_;

    bool checkVelocity(double vel);
    bool checkDeceleration(double pos, double vel);
    bool checkPosition(double pos, double vel);

//    double getAcceleration();
    double getDeceleration();

//    void updateVelocityMode(double dt);
//    void updatePositionMode(double dt);
};

#endif // RAMP_GENERATOR_H
