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

#ifndef LUH_YOUBOT_KINEMATICS_H
#define LUH_YOUBOT_KINEMATICS_H

#include <string>
#include <vector>
#include <math.h>
#include <luh_youbot_msgs/CartesianVector.h>
#include <tf/transform_listener.h>

/// Kinematics namespace
namespace luh_youbot_kinematics
{

// class declarations
class GenericVector;
class JointVector;
class JointPosition;
class JointVelocity;
class JointAcceleration;
class CylindricVector;
class CylindricPosition;
class CylindricVelocity;
class CylindricAcceleration;
class CartesianVector;
class CartesianPosition;
class CartesianVelocity;
class CartesianAcceleration;

// non-class functions
/**
 * @brief Returns the lower bound of joint position range.
 * @return Lower bound of joint position range.
 */
JointPosition getMinJointPositions();

/**
 * @brief Returns the upper bound of joint position range.
 * @return Upper bound of joint position range.
 */
JointPosition getMaxJointPositions();

/**
 * @brief Returns the maximum value in input vector.
 * @param values Input vector.
 * @return Maximum value in input vector.
 */
double max(GenericVector const& values);

/**
 * @brief Returns the minimum value in input vector.
 * @param values Input vector.
 * @return Minimum value in input vector.
 */
double min(GenericVector const& values);

/**
 * @brief Returns a vector with absolute values of input vector.
 * @param values Input vector.
 * @return Vector with absolute values of input vector.
 */
GenericVector abs(GenericVector const& values);

// constants
/// number of joints
const unsigned int N_JOINTS = 5;

/// height of joint 2 (in relation to arm_link_0)
const double L0 = 0.115;

/// radial offset of joint 2 (in relation to axis of joint 1)
const double L1 = 0.033;

/// length of first arm segment
const double L2 = 0.155;

/// length of second arm segment
const double L3 = 0.135;

/// length of third arm segment including gripper
const double L4 = 0.230;

/// distance between arm_link_0 origin and axis of joint 1
const double ARM_LINK_0_X_OFFSET = 0.024;

/// maximum joint accelerations
//const double MAX_JNT_ACCELERATIONS[] = {0.7, 0.7, 0.7, 0.7, 0.7};
const double MAX_JNT_ACCELERATIONS[] = {2.0, 2.0, 2.0, 2.0, 2.0};

/// maximum joint velocities
//const double MAX_JNT_VELOCITIES[] = {0.8, 0.8, 0.8, 0.8, 0.8};
const double MAX_JNT_VELOCITIES[] = {2.6, 2.6, 2.6, 2.6, 1.3};

/// joint angle offsets (difference between desired and actual zero positions)
const double JOINT_OFFSETS[] =
{
    -2.949606436,
    -1.1344673701987218,
     2.5481772172532176,
    -1.7889600250839740,
    -3.0019466477485340
};

/// maximum joint positions (in relation to desired zero positions)
const double MAX_JNT_POSITIONS[] =
{
    2.8905337534,
    1.4835265078,
    2.532469254,
    1.6402432236,
    2.6396457936
};

/// minimum joint positions (in relation to desired zero positions)
const double MIN_JNT_POSITIONS[] =
{
    -2.9395372288,
    -1.124398163,
    -2.4783710285,
    -1.7668361332,
    -2.8913271881
};

// operators
const GenericVector operator-(GenericVector const& val1, GenericVector const& val2);
const GenericVector operator+(GenericVector const& val1, GenericVector const& val2);
const GenericVector operator*(GenericVector const& val1, GenericVector const& val2);
const GenericVector operator/(GenericVector const& val1, GenericVector const& val2);
const GenericVector operator*(double const& val1, GenericVector const& val2);
const GenericVector operator*(GenericVector const& val1, double const& val2);
const GenericVector operator/(GenericVector const& val1, double const& val2);


// ########################### GENERIC VECTOR COORDINATES ##############################################################
/**
 * @brief A generic vector class with 5 elements. Base class for position, velocity, etc.
 */
class GenericVector: public std::vector<double>
{    
public:
    /**
     * @brief Constructor initialises vector with zeros.
     */
    GenericVector();

    /**
     * @brief Set the values.
     * @param values std::vector containing the desired values.
     */
    void setValues(const std::vector<double> &values);

    /**
     * @brief Set the values
     * @param values Pointer to an array with the desired values.
     */
    void setValues(const double* values);

    /**
     * @brief Print the vector's values to output stream. Useful for debugging.
     * @param name This string will be printed above the values.
     */
    void printValues(std::string name) const;

    /**
     * @brief Returns the maximum element of the vector.
     * @return Maximum element.
     */
    double max()const;

    /**
     * @brief Returns the minimum element of the vector.
     * @return Minimum element.
     */
    double min()const;

    /**
     * @brief Returns a vector with absolute values of input vector.
     * @return Vector with absolute values of input vector.
     */
    GenericVector abs()const;

    /**
     * @brief Checks if vector elements are not NaN.
     * @return False if at least one element is NaN.
     */
    bool isValid()const;

    /**
     * @brief Checks if all elements are zero.
     * @return True if all elements are zero.
     */
    bool isZero()const;

    GenericVector& operator+=(GenericVector const& rhs);
    GenericVector& operator-=(GenericVector const& rhs);
    GenericVector& operator*=(double const& rhs);
    GenericVector& operator/=(double const& rhs);
    GenericVector& operator*=(GenericVector const& rhs);
    GenericVector& operator/=(GenericVector const& rhs);
};

// ########################### CARTESIAN COORDINATES ###################################################################
/**
 * @brief Base class for cartesian vectors.
 */
class CartesianVector: public GenericVector
{
public:

    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CartesianVector();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CartesianVector(std::vector<double> val);

    /**
     * @brief Get value of x coordinate.
     * @return Value of x coordinate.
     */
    inline double x()    const{ return (*this)[0];}

    /**
     * @brief Get value of y coordinate.
     * @return Value of y coordinate.
     */
    inline double y()    const{ return (*this)[1];}

    /**
     * @brief Get value of z coordinate.
     * @return Value of z coordinate.
     */
    inline double z()    const{ return (*this)[2];}

    /**
     * @brief Get value of theta coordinate.
     * @return Value of theta coordinate.
     */
    inline double theta()const{ return (*this)[3];}

    /**
     * @brief Get value of q5 coordinate.
     * @return Value of q5 coordinate.
     */
    inline double q5()   const{ return (*this)[4];}

    /**
     * @brief Set value of x coordinate.
     * @param Value of x coordinate.
     */
    inline void setX(double x){(*this)[0] = x;}

    /**
     * @brief Set value of y coordinate.
     * @param Value of y coordinate.
     */
    inline void setY(double y){(*this)[1] = y;}

    /**
     * @brief Set value of z coordinate.
     * @param Value of z coordinate.
     */
    inline void setZ(double z){(*this)[2] = z;}

    /**
     * @brief Set value of theta coordinate.
     * @param Value of theta coordinate.
     */
    inline void setTheta(double theta){(*this)[3] = theta;}

    /**
     * @brief Set value of q5 coordinate.
     * @param Value of q5 coordinate.
     */
    inline void setQ5(double q5){(*this)[4] = q5;}

};

/**
 * @brief A vector to represent a cartesian position.
 */
class CartesianPosition: public CartesianVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CartesianPosition();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CartesianPosition(std::vector<double> val);

    /**
     * @brief Transform to jointspace coordinates (inverse kinematics) using reference position.
     * @param reference_position Reference position to solve ambiguities. This can e.g. be the current position.
     * @param same_config Specifies how the arm configuration is derived from the reference position.
     * @return Joint positions.
     * @details The arm configuration can be 'elbow up' or 'elbow down' and 'overhead' or 'head-on'. If the parameter
     *          'same_config' is true, the same arm configuration as in the reference position will be chosen. Otherwise
     *          all configurations will be calculated and the position closest to the reference position will be chosen.
     */
    JointPosition toJointspace(const JointPosition &reference_position, bool same_config=false)const;

    /**
     * @brief Transform to jointspace coordinates (inverse kinematics) with direct specification of configuration.
     * @param elbow_down Determines if elbow is up or down.
     * @param overhead Determines if the arm is in an overhead configuration.
     * @return Joint positions.
     */
    JointPosition toJointspace(bool elbow_down, bool overhead)const;

    /**
     * @brief Transform to jointspace coordinates (inverse kinematics) using heuristics to solve ambiguities.
     * @details The Method returns a valid solution if one exists. If more than one valid solutions exist heuristics
     *          are used to find the "best" configuration.
     * @return Joint positions.
     */
    JointPosition toJointspace()const;

    /**
     * @brief Transform to cylindrical coordinates with direct specification of configuration.
     * @param overhead Determines if the arm is in an overhead configuration.
     * @return Cylindric position.
     */
    CylindricPosition toCylindric(bool overhead)const;

    /**
     * @brief Transform to cylindrical coordinates using reference position.
     * @param reference_position Reference position to solve ambiguities. This can e.g. be the current position.
     * @param same_config Specifies how the arm configuration is derived from the reference position.
     * @return Cylindric position.
     * @details The arm configuration can be 'overhead' or 'head-on'. If the parameter 'same_config'
     *          is true, the same arm configuration as in the reference position will be chosen. Otherwise all
     *          configurations will be calculated and the position closest to the reference position will be chosen.
     */
    CylindricPosition toCylindric(const JointPosition &reference_position, bool same_config=false)const;

    /**
     * @brief Transform to cylindrical coordinates using heuristics to solve ambiguities.
     * @details The Method returns a valid solution if one exists. If more than one valid solutions exist heuristics
     *          are used to find the "best" configuration.
     * @return Cylindric position.
     */
    CylindricPosition toCylindric()const;

    /**
     * @brief Coordinate transformation from base frame to TCP frame.
     * @param current_position The current position.
     */
    void transformFromBaseToTCP(const JointPosition &current_position);

    /**
     * @brief Coordinate transformation from TCP frame to base frame.
     * @param current_position The current position.
     */
    void transformFromTCPToBase(const JointPosition &current_position);

    /**
     * @brief Converts ROS-Message to cartesian position vector.
     * @param pose_msg The ROS-Message.
     * @param is_relative Specifies if the pose is relative to current position or absolute.
     * @param joint_state_position Current joint state position is needed if the pose is relative.
     * @param tf_listener Pointer to TransformListener needed for transformation.
     * @return Cartesian position.
     */
    static CartesianPosition fromMsg(const luh_youbot_msgs::CartesianVector &pose_msg,
                                     bool is_relative, const JointPosition &joint_state_position,
                                     const tf::TransformListener* tf_listener);

    /**
     * @brief Checks if the position is reachable by the arm. Does not check joint range limitations or self collisions.
     * @return True if position is reachable.
     */
    bool isReachable();

};

/**
 * @brief A vector to represent a cartesian velocity.
 */
class CartesianVelocity: public CartesianVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CartesianVelocity();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CartesianVelocity(std::vector<double> val);

    /**
     * @brief Transform to jointspace coordinates (differential kinematics).
     * @param joint_positions The current joint positions.
     * @return Velocity in jointspace.
     */
    JointVelocity toJointspace(const JointPosition &joint_positions)const;

    /**
     * @brief Transform to cylindrical coordinates.
     * @param position The current cylincric position.
     * @return Velocity in cylindrical coordinates.
     */
    CylindricVelocity toCylindric(const CylindricPosition &position)const;

    /**
     * @brief Coordinate transformation from base frame to TCP frame.
     * @param current_position The current joint positions.
     */
    void transformFromBaseToTCP(const JointPosition &current_position);

    /**
     * @brief Coordinate transformation from TCP frame to base frame.
     * @param current_position The current joint positions.
     */
    void transformFromTCPToBase(const JointPosition &current_position);

    /**
     * @brief Converts ROS-Message to cartesian velocity vector.
     * @param velocity_msg The ROS-Message.
     * @param tf_listener Pointer to TransformListener needed for transformation.
     * @return Cartesian velocity.
     */
    static CartesianVelocity fromMsg(const luh_youbot_msgs::CartesianVector &velocity_msg,
                                     const tf::TransformListener *tf_listener);
};

/**
 * @brief A vector to represent a cartesian acceleration.
 */
class CartesianAcceleration: public CartesianVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CartesianAcceleration();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CartesianAcceleration(std::vector<double> val);

};

// ########################### CYLINDRICAL COORDINATES #################################################################
/**
 * @brief Base class for cylindrical vectors.
 */
class CylindricVector: public GenericVector
{
public:

    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CylindricVector();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CylindricVector(std::vector<double> val);

    /**
     * @brief Get value of q1 coordinate.
     * @return Value of q1 coordinate.
     */
    inline double q1()   const{ return (*this)[0];}

    /**
     * @brief Get value of r coordinate.
     * @return Value of r coordinate.
     */
    inline double r()    const{ return (*this)[1];}

    /**
     * @brief Get value of z coordinate.
     * @return Value of z coordinate.
     */
    inline double z()    const{ return (*this)[2];}

    /**
     * @brief Get value of theta coordinate.
     * @return Value of theta coordinate.
     */
    inline double theta()const{ return (*this)[3];}

    /**
     * @brief Get value of q5 coordinate.
     * @return Value of q5 coordinate.
     */
    inline double q5()   const{ return (*this)[4];}

    /**
     * @brief Set value of q1 coordinate.
     * @param Value of q1 coordinate.
     */
    inline void setQ1(double q1){(*this)[0] = q1;}

    /**
     * @brief Set value of r coordinate.
     * @param Value of r coordinate.
     */
    inline void setR(double r){(*this)[1] = r;}    

    /**
     * @brief Set value of z coordinate.
     * @param Value of z coordinate.
     */
    inline void setZ(double z){(*this)[2] = z;}

    /**
     * @brief Set value of theta coordinate.
     * @param Value of theta coordinate.
     */
    inline void setTheta(double theta){(*this)[3] = theta;}

    /**
     * @brief Set value of q5 coordinate.
     * @param Value of q5 coordinate.
     */
    inline void setQ5(double q5){(*this)[4] = q5;}
};

/**
 * @brief A vector to represent a cylindrical position.
 */
class CylindricPosition: public CylindricVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CylindricPosition();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CylindricPosition(std::vector<double> val);

    /**
     * @brief Transform to jointspace coordinates (inverse kinematics) using reference position.
     * @param reference_position Reference position to solve ambiguities. This can e.g. be the current position.
     * @param same_config Specifies how the configuration is derived from the reference position.
     * @return Joint positions.
     * @details The arm configuration can be 'elbow up' or 'elbow down'. If the parameter 'same_config'
     *          is true, the same arm configuration as in the reference position will be chosen. Otherwise all
     *          configurations will be calculated and the position closest to the reference position will be chosen.
     */
    JointPosition toJointspace(const JointPosition &reference_position, bool same_config=false)const;

    /**
     * @brief Transform to jointspace coordinates (inverse kinematics) with direct specification of configuration.
     * @param elbow_down Determines if the elbow is up or down.
     * @return Joint positions.
     */
    JointPosition toJointspace(bool elbow_down)const;

    /**
     * @brief Transform to cylindrical coordinates using heuristics to solve ambiguities.
     * @details The Method returns a valid solution if one exists. If more than one valid solutions exist heuristics
     *          are used to find the "best" configuration.
     * @return Joint positions.
     */
    JointPosition toJointspace()const;

    /**
     * @brief Transform to cartesian coordinates.
     * @return Cartesian position.
     */
    CartesianPosition toCartesian() const;

    /**
     * @brief Checks if the position is reachable by the arm. Does not check joint range limitations or self collisions.
     * @return True if position is reachable.
     */
    bool isReachable();

};

/**
 * @brief A vector to represent a cylindrical position.
 */
class CylindricVelocity: public CylindricVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CylindricVelocity();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CylindricVelocity(std::vector<double> val);

    /**
     * @brief Transform to cartesian coordiantes.
     * @param position The current position.
     * @return Cartesian velocity.
     */
    CartesianVelocity toCartesian(const CylindricPosition &position)const;

    /**
     * @brief Transform to jointspace coordinates.
     * @param joint_positions The current joint positions.
     * @return Joint velocities.
     */
    JointVelocity toJointspace(const JointPosition &joint_positions)const;

};

/**
 * @brief A vector to represent a cylindrical acceleration.
 */
class CylindricAcceleration: public CylindricVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    CylindricAcceleration();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    CylindricAcceleration(std::vector<double> val);

    /**
     * @brief Transform to cartesian coordinates.
     * @param velocity The current velocity.
     * @param position The current position.
     * @return Cartesian Acceleration.
     */
    CartesianAcceleration toCartesian(const CylindricVelocity &velocity, const CylindricPosition &position)const;

};

// ########################### JOINT COORDINATES #######################################################################
/**
 * @brief Base class for jointspace vectors.
 */
class JointVector : public GenericVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    JointVector();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    JointVector(std::vector<double> val);

    /**
     * @brief Set value of q1 coordinate.
     * @param Value of q1 coordinate.
     */
    inline double q1()const{ return (*this)[0];}

    /**
     * @brief Set value of q2 coordinate.
     * @param Value of q2 coordinate.
     */
    inline double q2()const{ return (*this)[1];}

    /**
     * @brief Set value of q3 coordinate.
     * @param Value of q3 coordinate.
     */
    inline double q3()const{ return (*this)[2];}

    /**
     * @brief Set value of q4 coordinate.
     * @param Value of q4 coordinate.
     */
    inline double q4()const{ return (*this)[3];}

    /**
     * @brief Set value of q5 coordinate.
     * @param Value of q5 coordinate.
     */
    inline double q5()const{ return (*this)[4];}

    /**
     * @brief Set value of q1 coordinate.
     * @param Value of q1 coordinate.
     */
    inline void setQ1(double q1){(*this)[0] = q1;}

    /**
     * @brief Set value of q2 coordinate.
     * @param Value of q2 coordinate.
     */
    inline void setQ2(double q2){(*this)[1] = q2;}

    /**
     * @brief Set value of q3 coordinate.
     * @param Value of q3 coordinate.
     */
    inline void setQ3(double q3){(*this)[2] = q3;}

    /**
     * @brief Set value of q4 coordinate.
     * @param Value of q4 coordinate.
     */
    inline void setQ4(double q4){(*this)[3] = q4;}

    /**
     * @brief Set value of q5 coordinate.
     * @param Value of q5 coordinate.
     */
    inline void setQ5(double q5){(*this)[4] = q5;}
};

/**
 * @brief A vector to represent a position in jointspace.
 */
class JointPosition : public JointVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    JointPosition();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    JointPosition(const std::vector<double> &val);

    /**
     * @brief Transform to cylindrical coordinates (forward kinematics).
     * @return Cylindrical Position.
     */
    CylindricPosition toCylindric()const;

    /**
     * @brief Transform to cartesian coordinates (forward kinematics).
     * @return Cartesian Position.
     */
    CartesianPosition toCartesian() const;

    /**
     * @brief Adds an offset to convert raw joint angles to angles with defined zero positions.
     */
    void addOffset();

    /**
     * @brief Subtracts an offset to convert angles with defined zero positions back to raw joint angles.
     */
    void subtractOffset();

    /**
     * @brief Checks if all joint angles are within the joint limitations.
     * @return True if position is reachable.
     */
    bool isReachable();

};

/**
 * @brief A vector to represent a velocity in jointspace.
 */
class JointVelocity : public JointVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    JointVelocity();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    JointVelocity(const std::vector<double> &val);

    /**
     * @brief Transform to cylindrical coordinates.
     * @param joint_position The current joint positions.
     * @return Cylindrical velocity.
     */
    CylindricVelocity toCylindric(const JointPosition &joint_position)const;

    /**
     * @brief Transform to cartesian coordinates.
     * @param joint_position The current joint positions.
     * @return Cartesian velocity.
     */
    CartesianVelocity toCartesian(const JointPosition &joint_position)const;

};

/**
 * @brief A vector to represent a acceleration in jointspace.
 */
class JointAcceleration : public JointVector
{
public:
    /**
     * @brief Empty constructor initialises vector with zeros.
     */
    JointAcceleration();

    /**
     * @brief Constructor initialises vector from std::vector.
     * @param val std::vector containing at least 5 elements.
     */
    JointAcceleration(const std::vector<double> &val);

};

} // namespace luh_youbot_kinematics

#endif // LUH_YOUBOT_KINEMATICS_H
