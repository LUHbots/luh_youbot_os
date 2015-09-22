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
 * @brief  Header file of luh_youbot_kinematics package.
 */

#ifndef LUH_YOUBOT_DYNAMICS_H
#define LUH_YOUBOT_DYNAMICS_H

#include <luh_youbot_kinematics/arm_kinematics.h>

namespace luh_youbot_kinematics
{

class JointEffort;
class CylindricEffort;
class CartesianEffort;
typedef CylindricEffort CylindricWrench;
typedef CartesianEffort CartesianWrench;

/**
 * @brief A vector to represent joint efforts (torques)
 */
class JointEffort : public JointVector
{
public:
    /**
     * @brief Constructor initialises vector with zeros.
     */
    JointEffort();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    JointEffort(const std::vector<double> &val);

    /**
     * @brief Transforms joint efforts to cylindric coordinates (torques and forces)
     * @param joint_position Current joint positions.
     * @return Forces and Torques in cylindrical coordinates.
     */
    CylindricEffort toCylindric(JointPosition joint_position);

    /**
     * @brief Transforms joint efforts to cartesian coordinates (torques and forces)
     * @param joint_position Current joint positions.
     * @return Forces and Torques in cartesian coordinates.
     */
    CartesianEffort toCartesian(JointPosition joint_position);
};

/**
 * @brief A vector to represent efforts in cylindrical coordinates (torques and forces)
 */
class CylindricEffort : public CylindricVector
{
public:
    /**
     * @brief Constructor initialises vector with zeros.
     */
    CylindricEffort();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CylindricEffort(const std::vector<double> &val);

    /**
     * @brief Transforms cylindric efforts to jointspace (torques)
     * @param joint_position Current joint positions.
     * @return Joint torques.
     */
    JointEffort toJointspace(JointPosition joint_position);

    /**
     * @brief Transforms cylindric efforts to cartesian coordinates (torques and forces)
     * @param joint_position Current joint positions.
     * @return Forces and Torques in cartesian coordinates.
     */
    CartesianEffort toCartesian(JointPosition joint_position);
};

/**
 * @brief A vector to represent efforts in cartesian coordinates (torques and forces)
 */
class CartesianEffort : public CartesianVector
{
public:
    /**
     * @brief Constructor initialises vector with zeros.
     */
    CartesianEffort();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CartesianEffort(const std::vector<double> &val);

    /**
     * @brief Transforms cartesian efforts to jointspace (torques)
     * @param joint_position Current joint positions.
     * @return Joint torques.
     */
    CylindricEffort toCylindric(JointPosition joint_position);

    /**
     * @brief Transforms cartesian efforts to jointspace (torques)
     * @param joint_position Current joint positions.
     * @return Joint torques.
     */
    JointEffort toJointspace(JointPosition joint_position);
};

// dynamics calculation still has some issues to be fixed
//struct DynamicParameters
//{
//    // friction koefficients
//    JointVector friction_p_1;
//    JointVector friction_p_2;
//    JointVector friction_p_3;
//    JointVector friction_n_1;
//    JointVector friction_n_2;
//    JointVector friction_n_3;

//    // mass/inertia
//    JointVector mass;
//    JointVector inertia_x;
//    JointVector inertia_y;
//    JointVector inertia_z;

//    // center of mass
//    JointVector com_radius;
//    JointVector com_angle;

//    // gravity
//    double g;
//};

/**
 * @brief This struct holds the manipulator's static parameters
 *
 * The centers of mass are given in polar coordinates relative to the corresponding joint
 *
 */
struct StaticParameters
{
    // masses
    /// mass of segment 5
    double mass_5;
    /// mass of segment 4
    double mass_4;
    /// mass of segment 3
    double mass_3;
    /// mass of segment 2
    double mass_2;

    // center of mass
    /// center of mass radius of segment 5
    double com_radius_5;
    /// center of mass radius of segment 4
    double com_radius_4;
    /// center of mass radius of segment 3
    double com_radius_3;
    /// center of mass radius of segment 2
    double com_radius_2;
    /// center of mass angle offset of segment 5
    double com_angle_5;
    /// center of mass angle offset of segment 4
    double com_angle_4;
    /// center of mass angle offset of segment 3
    double com_angle_3;
    /// center of mass angle offset of segment 2
    double com_angle_2;

    // friction
    /// friction of joint 5
    double friction_5;
    /// friction of joint 4
    double friction_4;
    /// friction of joint 3
    double friction_3;
    /// friction of joint 2
    double friction_2;

    // gravity
    /// gravity
    double gravity;
};

/**
 * @brief This class contains the static and dynamic model of the youbot's manipulator
 */
class YoubotArmDynamics
{
public:
    /**
     * @brief Empty constructor.
     */
    YoubotArmDynamics();

//    void setJointPosition(JointPosition pos){position_ = pos;}
//    void setJointVelocity(JointVelocity vel){velocity_ = vel;}
//    void setJointAcceleration(JointAcceleration acc){acceleration_ = acc;}


    /**
     * @brief Calculate the static joint effort in the given position.
     * @param Arm position in jointspace.
     * @return Joint torques.
     */
    JointEffort getStaticJointEffort(JointPosition pos);

//    JointEffort getDynamicJointEffort();
//    JointEffort getDynamicJointEffort(CylindricWrench wrench);
//    JointEffort getDynamicJointEffort(CartesianWrench wrench);

//    void setDynamicParameters(DynamicParameters parameters){p_ = parameters;}
    /**
     * @brief Set the static parameters.
     * @param parameters Static parameters.
     */
    void setStaticParameters(StaticParameters parameters){stat_p_ = parameters;}

protected:
    JointPosition position_;
    JointVelocity velocity_;
    JointAcceleration acceleration_;

//    DynamicParameters dyn_p_;
    StaticParameters stat_p_;
};

} // namespace luh_youbot_kinematics

#endif // LUH_YOUBOT_DYNAMICS_H
