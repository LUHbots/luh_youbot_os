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
 * @brief  Source file for all cartesian vectors.
 */

#include "luh_youbot_kinematics/arm_kinematics.h"
#include "ros/assert.h"
#include "tf/tf.h"

namespace luh_youbot_kinematics
{

// ######################## CONSTRUCTORS ####################################################
CartesianVector::CartesianVector(): GenericVector(){}
CartesianPosition::CartesianPosition(): CartesianVector(){}
CartesianVelocity::CartesianVelocity(): CartesianVector(){}
CartesianAcceleration::CartesianAcceleration(): CartesianVector(){}

CartesianVector::CartesianVector(std::vector<double> val): GenericVector()
{
    this->setValues(val);
}
CartesianPosition::CartesianPosition(std::vector<double> val): CartesianVector()
{
    this->setValues(val);
}
CartesianVelocity::CartesianVelocity(std::vector<double> val): CartesianVector()
{
    this->setValues(val);
}
CartesianAcceleration::CartesianAcceleration(std::vector<double> val): CartesianVector()
{
    this->setValues(val);
}

// ######################## CARTESIAN POSITION TO JOINTSPACE ###########################################################
JointPosition CartesianPosition::toJointspace(const JointPosition &reference_position, bool same_config) const
{
    if(same_config)
    {
        // the sign of joint 3 determines if the elbow is up or down
        bool elbow_down = (reference_position.q3() < 0);

        // if the radius in cylindrical coordinates is negative, the arm is in an overhead configuration
        bool overhead = (reference_position.toCylindric().r() < 0);

        return this->toJointspace(elbow_down, overhead);

    }
    else
    {
        // 4 possible configuration combinations
        JointPosition conf1 = this->toJointspace(true, true);
        JointPosition conf2 = this->toJointspace(false, false);
        JointPosition conf3 = this->toJointspace(false, true);
        JointPosition conf4 = this->toJointspace(true, false);

        double err1 = 0;
        double err2 = 0;
        double err3 = 0;
        double err4 = 0;

        double d;

        // calculate the distances to reference position
        for(uint i=0; i<N_JOINTS; i++)
        {
            d = conf1[i] - reference_position[i];
            err1 += d*d;
            d = conf2[i] - reference_position[i];
            err2 += d*d;
            d = conf3[i] - reference_position[i];
            err3 += d*d;
            d = conf4[i] - reference_position[i];
            err4 += d*d;
        }

        if(!(conf1.isReachable() && conf1.isValid()))
            err1 = HUGE_VAL;
        if(!(conf2.isReachable() && conf2.isValid()))
            err2 = HUGE_VAL;
        if(!(conf3.isReachable() && conf3.isValid()))
            err3 = HUGE_VAL;
        if(!(conf4.isReachable() && conf4.isValid()))
            err4 = HUGE_VAL;

        // return configuration with minimum distance
        if(err1 < err2)
        {
            if(err1 < err3)
            {
                if(err1 < err4)
                    return conf1;
                else return conf4;
            }
            else
            {
                if(err3 < err4)
                    return conf3;
                else return conf4;
            }
        }
        else
        {
            if(err2 < err3)
            {
                if(err2 < err4)
                    return conf2;
                else return conf4;
            }
            else
            {
                if(err3 < err4)
                    return conf3;
                else return conf4;
            }
        }
    }
}

JointPosition CartesianPosition::toJointspace(bool elbow_down, bool overhead) const
{
    return this->toCylindric(overhead).toJointspace(elbow_down);
}

JointPosition CartesianPosition::toJointspace() const
{
    return this->toCylindric().toJointspace();
}

// ######################## CARTESIAN VELOCITY TO JOINTSPACE ###########################################################
JointVelocity CartesianVelocity::toJointspace(const JointPosition &joint_positions) const
{
    CylindricPosition p = joint_positions.toCylindric();
    return this->toCylindric(p).toJointspace(joint_positions);
}

// ######################## CARTESIAN POSITION TO CYLINDRIC ############################################################
CylindricPosition CartesianPosition::toCylindric(bool overhead) const
{
    // transform to cylindrical coordinates
    CylindricPosition cyl_pos;
    double x = this->x() - ARM_LINK_0_X_OFFSET;
    double r = sqrt(x*x + this->y()*this->y());
    double q1 = -atan2(this->y(), x);
    cyl_pos.setZ(this->z());
    cyl_pos.setTheta(this->theta());
    cyl_pos.setQ5(this->q5());

    // to get an overhead configuration, the radius must be negative and joint 1 must be turned +-180°
    if(overhead)
    {
        r *= -1;
        if(q1 > 0)
            q1 -= M_PI;
        else
            q1 += M_PI;
    }

    cyl_pos.setR(r);
    cyl_pos.setQ1(q1);

    return cyl_pos;
}

CylindricPosition CartesianPosition::toCylindric() const
{
    // if no configuration is given it is assumed that the endeffector is facing away from the axis of joint 1
    // in that case a negative theta would lead to an overhead configuration
    bool overhead = (this->theta() < 0);

    CylindricPosition cyl_pos = this->toCylindric(overhead);

    // if the chosen configuration is not reachable the other one is used
    if(!cyl_pos.isReachable() || !cyl_pos.toJointspace().isReachable())
    {
        return this->toCylindric(!overhead);
    }
    else
        return cyl_pos;
}

CylindricPosition CartesianPosition::toCylindric(const JointPosition &reference_position, bool same_config) const
{
    if(same_config)
    {
        // if the radius in cylindrical coordinates is negative, the arm is in an overhead configuration
        bool overhead = (reference_position.toCylindric().r() < 0);

        return this->toCylindric(overhead);
    }
    else
    {
        CylindricPosition conf1 = this->toCylindric(true);
        CylindricPosition conf2 = this->toCylindric(false);

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

        if(err1 < err2 && conf1.isReachable() && conf1.isValid())
            return conf1;
        else return conf2;
    }
}

// ######################## CARTESIAN VELOCITY TO CYLINDRIC ############################################################
CylindricVelocity CartesianVelocity::toCylindric(const CylindricPosition &position) const
{
//    CylindricVelocity v;
//    CartesianPosition cart_pos = position.toCartesian();
//    double x = cart_pos.x() - ARM_LINK_0_X_OFFSET;

//    double r_2 = position.r() * position.r();

//    v.setR((x * this->x() + cart_pos.y() * this->y()) / (sqrt(r_2)));
//    v.setQ1((cart_pos.y() * this->x() - x * this->y()) / r_2);
//    v.setZ(this->z());
//    v.setTheta(this->theta());
//    v.setQ5(this->q5());

//    // invert radial velocity in overhead configuration
//    if(position.r() < 0)
//        v.setR(v.r() * -1);

//    return v;

    CylindricVelocity v;

    double sq1 = sin(position.q1());
    double cq1 = cos(position.q1());

    v.setR(this->x() * cq1 - this->y() * sq1);
    v.setQ1((-this->x() * sq1 - this->y() * cq1) / position.r());
    v.setZ(this->z());
    v.setTheta(this->theta());
    v.setQ5(this->q5());

    return v;
}

// ######################## TRANSFORMATIONS ############################################################################
void CartesianPosition::transformFromBaseToTCP(const JointPosition &current_position)
{
    // current pose
    CartesianPosition cart_pos = current_position.toCartesian();

    // cos and sin
    double ctheta = cos(cart_pos.theta());
    double stheta = sin(cart_pos.theta());
    double cq5 = cos(cart_pos.q5());
    double sq5 = sin(cart_pos.q5());
    double cq1 = cos(current_position.q1());
    double sq1 = sin(current_position.q1());

    // translation
    this->setX(this->x() - cart_pos.x());
    this->setY(this->y() - cart_pos.y());
    this->setZ(this->z() - cart_pos.z());

    // q1 rotation
    double x2 = this->x() * cq1 - this->y() * sq1;
    double y2 = this->x() * sq1 + this->y() * cq1;
    double z2 = this->z();

    // theta rotation
    double x1 = z2 * ctheta + x2 * stheta;
    double y1 = y2;
    double z1 = z2 * stheta - x2 * ctheta;

    // q5 rotation
    this->setX(z1 * cq5 + y1 * sq5);
    this->setY(z1 * sq5 - y1 * cq5);
    this->setZ(x1);
}

void CartesianPosition::transformFromTCPToBase(const JointPosition &current_position)
{
    // current pose
    CartesianPosition cart_pos = current_position.toCartesian();

    // cos and sin
    double ctheta = cos(cart_pos.theta());
    double stheta = sin(cart_pos.theta());
    double cq5 = cos(cart_pos.q5());
    double sq5 = sin(cart_pos.q5());
    double cq1 = cos(current_position.q1());
    double sq1 = sin(current_position.q1());

    // q5 rotation
    double x1 = this->z();
    double y1 = this->x() * sq5 - this->y() * cq5;
    double z1 = this->x() * cq5 + this->y() * sq5;

    // theta rotation
    double x2 = x1 * stheta - z1 * ctheta;
    double y2 = y1;
    double z2 = x1 * ctheta + z1 * stheta;

    // q1 rotation
    this->setX(y2 * sq1 + x2 * cq1);
    this->setY(y2 * cq1 - x2 * sq1);
    this->setZ(z2);

    //translation
    this->setX(this->x() + cart_pos.x());
    this->setY(this->y() + cart_pos.y());
    this->setZ(this->z() + cart_pos.z());

}

void CartesianVelocity::transformFromBaseToTCP(const JointPosition &current_position)
{
    // current pose
    CartesianPosition cart_pos = current_position.toCartesian();

    // cos and sin
    double ctheta = cos(cart_pos.theta());
    double stheta = sin(cart_pos.theta());
    double cq5 = cos(cart_pos.q5());
    double sq5 = sin(cart_pos.q5());
    double cq1 = cos(current_position.q1());
    double sq1 = sin(current_position.q1());

    // q1 rotation
    double x2 = this->x() * cq1 - this->y() * sq1;
    double y2 = this->x() * sq1 + this->y() * cq1;
    double z2 = this->z();

    // theta rotation
    double x1 = z2 * ctheta + x2 * stheta;
    double y1 = y2;
    double z1 = z2 * stheta - x2 * ctheta;

    // q5 rotation
    this->setX(z1 * cq5 + y1 * sq5);
    this->setY(z1 * sq5 - y1 * cq5);
    this->setZ(x1);
}

void CartesianVelocity::transformFromTCPToBase(const JointPosition &current_position)
{
    // current pose
    CartesianPosition cart_pos = current_position.toCartesian();

    // cos and sin
    double ctheta = cos(cart_pos.theta());
    double stheta = sin(cart_pos.theta());
    double cq5 = cos(cart_pos.q5());
    double sq5 = sin(cart_pos.q5());
    double cq1 = cos(current_position.q1());
    double sq1 = sin(current_position.q1());

    // q5 rotation
    double x1 = this->z();
    double y1 = this->x() * sq5 - this->y() * cq5;
    double z1 = this->x() * cq5 + this->y() * sq5;

    // theta rotation
    double x2 = x1 * stheta - z1 * ctheta;
    double y2 = y1;
    double z2 = x1 * ctheta + z1 * stheta;

    // q1 rotation
    this->setX(y2 * sq1 + x2 * cq1);
    this->setY(y2 * cq1 - x2 * sq1);
    this->setZ(z2);
}

// ######################## FROM ROS MESSAGE ###########################################################################
CartesianPosition CartesianPosition::fromMsg(const luh_youbot_msgs::CartesianVector &pose_msg,
                                             bool is_relative,
                                             const JointPosition &joint_state_position,
                                             const tf::TransformListener *tf_listener)
{
    CartesianPosition position;

    if(is_relative)
    {
        // === TRANSFORM ===
        geometry_msgs::Vector3Stamped rel_pos;
        geometry_msgs::Vector3Stamped rel_pos_in_base_frame;
        rel_pos.header = pose_msg.header;
        rel_pos.header.stamp = ros::Time(0);
        rel_pos.vector.x = pose_msg.x;
        rel_pos.vector.y = pose_msg.y;
        rel_pos.vector.z = pose_msg.z;

        if(rel_pos.header.frame_id.empty())
            rel_pos_in_base_frame = rel_pos;
        else
            tf_listener->transformVector("arm_link_0", rel_pos, rel_pos_in_base_frame);

        // === ADD TO CURRENT POSE ===
        position = joint_state_position.toCartesian();

        position.setX(position.x() + rel_pos_in_base_frame.vector.x);
        position.setY(position.y() + rel_pos_in_base_frame.vector.y);
        position.setZ(position.z() + rel_pos_in_base_frame.vector.z);
        position.setTheta(position.theta() + pose_msg.theta);
        position.setQ5(position.q5()+ pose_msg.q5);
    }
    else
    {
        // === TRANSFORM ===
        geometry_msgs::PointStamped abs_point;
        geometry_msgs::PointStamped abs_point_in_base_frame;
        abs_point.header = pose_msg.header;
        abs_point.header.stamp = ros::Time(0);
        abs_point.point.x = pose_msg.x;
        abs_point.point.y = pose_msg.y;
        abs_point.point.z = pose_msg.z;

        if(abs_point.header.frame_id.empty())
            abs_point_in_base_frame = abs_point;
        else
            tf_listener->transformPoint("arm_link_0", abs_point, abs_point_in_base_frame);

        position.setX(abs_point_in_base_frame.point.x);
        position.setY(abs_point_in_base_frame.point.y);
        position.setZ(abs_point_in_base_frame.point.z);
        position.setTheta(pose_msg.theta);
        position.setQ5(pose_msg.q5);
    }

    return position;
}

CartesianVelocity CartesianVelocity::fromMsg(const luh_youbot_msgs::CartesianVector &velocity_msg,
                                             const tf::TransformListener *tf_listener)
{
    CartesianVelocity velocity;
    velocity.setX(velocity_msg.x);
    velocity.setY(velocity_msg.y);
    velocity.setZ(velocity_msg.z);
    velocity.setTheta(velocity_msg.theta);
    velocity.setQ5(velocity_msg.q5);

    // only transform if a frame other that arm_link_0 is specified
    if(velocity_msg.header.frame_id.compare("arm_link_0") != 0 && !velocity_msg.header.frame_id.empty())
    {
        // === TRANSFORM ===
        geometry_msgs::Vector3Stamped gripper_vel;
        geometry_msgs::Vector3Stamped base_vel;
        gripper_vel.header = velocity_msg.header;
        gripper_vel.header.stamp = ros::Time(0);
        gripper_vel.vector.x = velocity_msg.x;
        gripper_vel.vector.y = velocity_msg.y;
        gripper_vel.vector.z = velocity_msg.z;

        tf_listener->transformVector("arm_link_0", gripper_vel, base_vel);

        velocity.setX(base_vel.vector.x);
        velocity.setY(base_vel.vector.y);
        velocity.setZ(base_vel.vector.z);
    }

    return velocity;
}

// ######################## VERIFICATION ###############################################################################
bool CartesianPosition::isReachable()
{
    return (this->toCylindric(true).isReachable() || this->toCylindric(false).isReachable());
}


} // namespace luh_youbot_kinematics





