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
 * @brief  Source file for generic vectors functions.
 */

#include "luh_youbot_kinematics/arm_kinematics.h"
#include <iostream>
#include "ros/assert.h"

namespace luh_youbot_kinematics
{

//########## NON CLASS FUNCTIONS #######################################################################################
JointPosition getMinJointPositions()
{
    std::vector<double> v(MIN_JNT_POSITIONS, MIN_JNT_POSITIONS + N_JOINTS);
    return JointPosition(v);
}

JointPosition getMaxJointPositions()
{
    std::vector<double> v(MAX_JNT_POSITIONS, MAX_JNT_POSITIONS + N_JOINTS);
    return JointPosition(v);
}

double max(GenericVector const& values)
{
    double max_val = 0;
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        max_val = std::max(max_val, values[i]);
    }
    return max_val;
}

double min(GenericVector const& values)
{
    double min_val = INFINITY;
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        min_val = std::min(min_val, values[i]);
    }
    return min_val;
}

GenericVector abs(GenericVector const& values)
{
    GenericVector v;
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        v[i] = std::fabs(values[i]);
    }
    return v;
}

// ############### CONSTRUCTOR #########################################################################################
GenericVector::GenericVector() : std::vector<double>()
{
    this->assign(N_JOINTS, 0.0);
}

// ############### SET VALUES ##########################################################################################
void GenericVector::setValues(const std::vector<double> &values)
{
    ROS_ASSERT(values.size() >= N_JOINTS);

    setValues(&values.front());
}

void GenericVector::setValues(const double *values)
{
    for(unsigned int i=0; i<N_JOINTS; i++)
        (*this)[i] = values[i];
}

// ############### MAX / MIN ###########################################################################################
double GenericVector::max() const
{
    return luh_youbot_kinematics::max(*this);
}

double GenericVector::min() const
{
    return luh_youbot_kinematics::min(*this);
}

// ############### ABS #################################################################################################
GenericVector GenericVector::abs() const
{
    return luh_youbot_kinematics::abs(*this);
}

// ############### IS VALID ############################################################################################
bool GenericVector::isValid() const
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        if(isnan((*this)[i]))
            return false;
    }
    return true;
}

// ############### IS ZERO #############################################################################################
bool GenericVector::isZero() const
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        if((*this)[i] != 0)
            return false;
    }
    return true;
}

// ############### PRINT VALUES ########################################################################################
void GenericVector::printValues(std::string name) const
{
    std::cout << name << ":" << std::endl;
    std::cout << "[";
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        if(i>0)
            std::cout << "; ";
        std::cout << (*this)[i];
    }
    std::cout << "]" << std::endl;
}

// ############### MEMBER OPERATORS ####################################################################################
GenericVector& GenericVector::operator+=(GenericVector const& rhs)
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] += rhs[i];
    }
    return *this;
}

GenericVector& GenericVector::operator-=(GenericVector const& rhs)
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] -= rhs[i];
    }
    return *this;
}
GenericVector& GenericVector::operator*=(double const& rhs)
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] *= rhs;
    }
    return *this;
}
GenericVector& GenericVector::operator/=(double const& rhs)
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] /= rhs;
    }
    return *this;
}
GenericVector& GenericVector::operator*=(GenericVector const& rhs)
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] *= rhs[i];
    }
    return *this;
}
GenericVector& GenericVector::operator/=(GenericVector const& rhs)
{
    for(unsigned int i=0; i<N_JOINTS; i++)
    {
        (*this)[i] /= rhs[i];
    }
    return *this;
}

// ############### STATIC OPERATORS ####################################################################################
const GenericVector operator-(GenericVector const& val1, GenericVector const& val2){
    GenericVector diff;
    for(unsigned int i=0; i<N_JOINTS; i++) diff[i] = val1[i] - val2[i];
    return diff;
}
const GenericVector operator+(GenericVector const& val1, GenericVector const& val2){
    GenericVector sum;
    for(unsigned int i=0; i<N_JOINTS; i++) sum[i] = val1[i] + val2[i];
    return sum;
}
const GenericVector operator*(GenericVector const& val1, GenericVector const& val2){
    GenericVector prod;
    for(unsigned int i=0; i<N_JOINTS; i++) prod[i] = val1[i] * val2[i];
    return prod;
}
const GenericVector operator/(GenericVector const& val1, GenericVector const& val2){
    GenericVector quot;
    for(unsigned int i=0; i<N_JOINTS; i++) quot[i] = val1[i] / val2[i];
    return quot;
}
const GenericVector operator*(double const& val1, GenericVector const& val2){
    GenericVector prod;
    for(unsigned int i=0; i<N_JOINTS; i++) prod[i] = val1 * val2[i];
    return prod;
}
const GenericVector operator*(GenericVector const& val1, double const& val2){

    return val2 * val1;
}
const GenericVector operator/(GenericVector const& val1, double const& val2){

    GenericVector quot;
    for(unsigned int i=0; i<N_JOINTS; i++) quot[i] = val1[i] / val2;
    return quot;
}

} // namespace luh_youbot_kinematics
