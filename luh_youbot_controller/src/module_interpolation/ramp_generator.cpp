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

#include "luh_youbot_controller/module_interpolation/ramp_generator.h"
#include <iostream>
#include <std_msgs/Float32.h>

namespace ykin = luh_youbot_kinematics;

//#################### CONSTRUCTOR #####################################################################################
RampGenerator::RampGenerator():
    got_target_(false), target_position_reached_(false)
{
    ros::NodeHandle nh;
    pub1 = nh.advertise<std_msgs::Float32>("debug/pos", 100);
    pub2 = nh.advertise<std_msgs::Float32>("debug/vel", 100);
    pub3 = nh.advertise<std_msgs::Float32>("debug/acc", 100);
    pub4 = nh.advertise<std_msgs::Float32>("debug/pos_soll", 100);
    pub5 = nh.advertise<std_msgs::Float32>("debug/vel_soll", 100);
}

//#################### UPDATE ##########################################################################################
void RampGenerator::update()
{
    if(!got_target_)
        return;

    target_position_reached_ = false;

    // === UPDATE TIME ===
    double new_time = ros::Time::now().toSec();
    delta_t_ = new_time - current_time_;
    current_time_ = new_time;

    // === CALCULATE NEXT TIME STEP ===
    double delta_v = current_.a * delta_t_;
    next_.s = current_.s + (current_.v + 0.5 * delta_v) * delta_t_;
    next_.v = current_.v + delta_v;
    last_ = current_;

    // === STATE MACHINE ===
    if(!velocity_mode_ && checkPosition(next_.s, next_.v))
    {
        setTargetVelocity(initial_velocity_);
        target_position_reached_ = true;

        if(initial_velocity_.isZero())
        {
            // set exact target position as current position
            current_.position = goal_position_;
            current_.velocity = initial_velocity_;
        }
    }
    else if(!velocity_mode_ && state_ != DECELERATE && checkDeceleration(next_.s, next_.v))
    {
        current_.a = getDeceleration();
        delta_v = current_.a * delta_t_;
        current_.s += (current_.v + 0.5 * delta_v) * delta_t_;
        current_.v += delta_v;

        state_ = DECELERATE;
    }
    else if(state_ == ACCELERATE && checkVelocity(next_.v))
    {
        current_.a = 0;
        delta_v = v_max_ - current_.v;
        current_.s += (current_.v + 0.5 * delta_v) * delta_t_;
        current_.v = v_max_;

        state_ = HOLD_VELOCITY;
    }
    else
    {
        current_.s = next_.s;
        current_.v = next_.v;
    }

    if(!target_position_reached_)
    {
        current_.position += delta_t_ * initial_velocity_ + (current_.s - last_.s) * vector_;
        current_.velocity = initial_velocity_ + current_.v * vector_;
    }

    // === DEBUG MSGS ===
    std_msgs::Float32 msg;
    msg.data = current_.s;
    pub1.publish(msg);

    msg.data = current_.v;
    pub2.publish(msg);

    msg.data = current_.a;
    pub3.publish(msg);

    msg.data = s_max_;
    pub4.publish(msg);

    msg.data = v_max_;
    pub5.publish(msg);
}

//#################### CHECK DECELERATION ##############################################################################
bool RampGenerator::checkDeceleration(double pos, double vel)
{
    // get necessary deceleration to reach goal state
    double deceleration = vel * vel / (2 * (s_max_ - pos));

    // check if deceleration is greater than max acceleration
    if (fabs(deceleration) > a_max_)
    {
        return true;
    }
    else return false;
}

//#################### CHECK VELOCITY ##################################################################################
bool RampGenerator::checkVelocity(double vel)
{
    return vel > v_max_;
}

//#################### CHECK POSITION ##################################################################################
bool RampGenerator::checkPosition(double pos, double vel)
{
    return pos > s_max_ || vel < 0;
}

//#################### GET DECELERATION ################################################################################
double RampGenerator::getDeceleration()
{
    return -current_.v * current_.v / (2 * (s_max_ - current_.s));
}

//#################### SET TARGET POSITION #############################################################################
void RampGenerator::setTargetPosition(ykin::GenericVector position, bool position_is_absolute)
{
    // === SET POSITION COMMAND ===
//    position_command_ = position;
    goal_position_ = position;

    if(position_is_absolute)
    {
        position -= current_.position;
    }
    else
    {
        goal_position_ += current_.position;
    }

    // === RESET CURRENT AND INITIAL VALUES ===
    current_.s = 0;

    initial_velocity_ = current_.velocity;
    initial_position_ = current_.position;

    current_.v = 0;

    s_max_ = 0;
    for(uint i=0; i<ykin::N_JOINTS; i++)
    {
        s_max_ += position[i] * position[i];
    }
    s_max_ = sqrt(s_max_);

    if(s_max_ != 0)
    {
        vector_ = position / s_max_;

        v_max_ = (max_velocity_ / vector_).abs().min();
        a_max_ = (max_acceleration_ / vector_).abs().min();
        current_.a = a_max_;
    }
    else
    {
        vector_ *= 0;

        v_max_ = 0;
        a_max_ = 0;
        current_.a = a_max_;
    }

    // === INITIALISE TIME AND FLAGS ===
    current_time_ = ros::Time::now().toSec();
    start_time_ = current_time_;
    got_target_ = true;
    velocity_mode_ = false;
    target_position_reached_ = false;
    state_ = ACCELERATE;
}

//#################### SET TARGET VELOCITY #############################################################################
void RampGenerator::setTargetVelocity(ykin::GenericVector velocity)
{
    //    velocity_command_ = velocity;
    initial_velocity_ = current_.velocity;
    initial_position_ = current_.position;
    current_.s = 0;
    current_.v = 0;

    // === CHECK LIMITS ===
    for(unsigned int i=0; i<ykin::N_JOINTS; i++)
    {
        velocity[i] = std::min(velocity[i], max_velocity_[i]);
        velocity[i] = std::max(velocity[i], -max_velocity_[i]);
    }    

    goal_velocity_ = velocity - current_.velocity;

    v_max_ = 0;
    for(uint i=0; i<ykin::N_JOINTS; i++)
    {
        v_max_ += goal_velocity_[i] * goal_velocity_[i];
    }
    v_max_ = sqrt(v_max_);

    if(v_max_ != 0)
    {
        vector_ = goal_velocity_ / v_max_;
        a_max_ = (max_acceleration_ / vector_).abs().min();
        current_.a = a_max_;
    }
    else
    {
        vector_ *= 0;
        a_max_ = 0;
        current_.a = 0;
    }

    // === INITIALISE TIME AND FLAGS ===
    current_time_ = ros::Time::now().toSec();
    start_time_ = current_time_;
    got_target_ = true;
    velocity_mode_ = true;
    target_position_reached_ = false;
    state_ = ACCELERATE;
}

//#################### GET VELOCITY ####################################################################################
ykin::GenericVector RampGenerator::getVelocity()
{
    return current_.velocity;
}

//#################### GET POSITION ####################################################################################
ykin::GenericVector RampGenerator::getPosition()
{
    return current_.position;
}

//#################### SET MAX VELOCITY ################################################################################
void RampGenerator::setMaxVelocity(const ykin::GenericVector &max_vel)
{
    max_velocity_ = max_vel;
}

//#################### SET MAX ACCELERATION ############################################################################
void RampGenerator::setMaxAcceleration(const ykin::GenericVector &max_acc)
{
    max_acceleration_ = max_acc;
}

//#################### RESET CURRENT STATE #############################################################################
void RampGenerator::resetCurrentState(const ykin::GenericVector &velocity, const ykin::GenericVector &position)
{
    current_.velocity = velocity;
    current_.position = position;
}

//#################### TARGET POSITION REACHED #########################################################################
bool RampGenerator::targetPositionReached()
{
    return target_position_reached_;
}

//#################### ADJUST VELOCITY #################################################################################
void RampGenerator::adjustVelocity(double factor)
{
    ykin::GenericVector delta_vi = (factor - 1) * initial_velocity_;
    double delta_v = (factor - 1) * current_.v;

    // reset current state
    current_ = last_;

    // recalculate with new velocity
    current_.s += (current_.v + 0.5 * delta_v) * delta_t_;
    current_.v += delta_v;
    initial_velocity_ *= factor;

    current_.position += delta_t_ * (initial_velocity_ + 0.5 * delta_vi) + (current_.s - last_.s) * vector_;
    current_.velocity = initial_velocity_ + current_.v * vector_;

    // change state and acceleration if necessary
    if(state_ == HOLD_VELOCITY)
    {
        current_.a = a_max_;
        state_ = ACCELERATE;
    }
    else if(state_ == DECELERATE)
    {
        current_.a = 0;
        state_ = HOLD_VELOCITY;
    }
}
