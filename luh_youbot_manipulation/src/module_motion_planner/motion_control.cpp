/* *****************************************************************
 *
 * imes_youbot_pkg
 *
 * Copyright (c) 2012,
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
 * @file   motionPlanning.cpp
 * @author Eduard Popp <eduardpopp@web.de>
 * @date   28. März 2013
 *
 * @brief  Diese Datei enthält die Klasse motionPlanning, die diverse Methoden zur Bahnplanung bereitstellt
 */
/* Modified by Simon Aden in 2014*/

#include "luh_youbot_manipulation/module_motion_planner/motion_control.hpp"

using namespace std;
using namespace luh_youbot_kinematics;

motionPlanning::motionPlanning() {

    LOOPRATE = 0.002;

    //Initialisieren der max. Geschwindigkeiten und Beschleunigungen
    JointProp.max_acc = MAX_JNT_ACCELERATIONS[0];    JointProp.max_vel = MAX_JNT_VELOCITIES[0];    JointProp.minAngle = MINANGLE1;     JointProp.maxAngle = MAXANGLE1;     initParamValues.push_back(JointProp);
    JointProp.max_acc = MAX_JNT_ACCELERATIONS[1];    JointProp.max_vel = MAX_JNT_VELOCITIES[1];    JointProp.minAngle = MINANGLE2;     JointProp.maxAngle = MAXANGLE2;     initParamValues.push_back(JointProp);
    JointProp.max_acc = MAX_JNT_ACCELERATIONS[2];    JointProp.max_vel = MAX_JNT_VELOCITIES[2];    JointProp.minAngle = MINANGLE3;     JointProp.maxAngle = MAXANGLE3;     initParamValues.push_back(JointProp);
    JointProp.max_acc = MAX_JNT_ACCELERATIONS[3];    JointProp.max_vel = MAX_JNT_VELOCITIES[3];    JointProp.minAngle = MINANGLE4;     JointProp.maxAngle = MAXANGLE4;     initParamValues.push_back(JointProp);
    JointProp.max_acc = MAX_JNT_ACCELERATIONS[4];    JointProp.max_vel = MAX_JNT_VELOCITIES[4];    JointProp.minAngle = MINANGLE5;     JointProp.maxAngle = MAXANGLE5;     initParamValues.push_back(JointProp);

    ePosSum = new double[JOINTSPACE];
    ePos = new double[JOINTSPACE];
    ePosAlt = new double[JOINTSPACE];

    busy = false;
    velocity_set = false;
    calculation_done = false;
    points = 0;
    timeIt = 0;
    gripperIt = 0;
    timeBaseIt = 0;

    t_f_base.resize(2);
    KP.resize(5);
    KI.resize(5);
    KD.resize(5);
    windUP.resize(5);
    stellwert.resize(10);

    //Initialisierung der Vektoren
    armJointVelocities.velocities.resize(ARMJOINTS);
    armJointPositions.positions.resize(ARMJOINTS);
    armJointAccelerations.accelerations.resize(ARMJOINTS);

    singleAxis.arm_axis.resize(ARMJOINTS);

    regler.positions.resize(15);
    baseParam.resize(3);
    base.arm_axis.resize(3);

    stringstream jointName;

    for (unsigned int idx = 0; idx < ARMJOINTS; ++idx) {
        jointName.str("");
        jointName << "arm_joint_" << (idx + 1);       

        armJointVelocities.velocities[idx].joint_uri = jointName.str();
        armJointVelocities.velocities[idx].unit = boost::units::to_string(boost::units::si::radians_per_second);
        armJointVelocities.velocities[idx].value = 0.0;

        armJointPositions.positions[idx].joint_uri = jointName.str();
        armJointPositions.positions[idx].unit = boost::units::to_string(boost::units::si::radians);
        armJointPositions.positions[idx].value = 0.0;

        armJointAccelerations.accelerations[idx].joint_uri = jointName.str();
        armJointAccelerations.accelerations[idx].unit = boost::units::to_string(boost::units::si::radians_per_second / boost::units::si::seconds);
        armJointAccelerations.accelerations[idx].value = 0.0;

        ePosSum[idx] = 0.0;
        ePos[idx] = 0.0;
        ePosAlt[idx] = 0.0;
    }

    goal_tolerance = 30 * M_PI/180;
    tolerance_check_percentage = 0.8;
}

motionPlanning::~motionPlanning()
{

}

void motionPlanning::setUpdateFrequency(double frequency)
{
    LOOPRATE = 1.0 / frequency;
}

void motionPlanning::init()
{
    //clear Time
    t_f_ppoint.clear();
    boolParam.clear();

    //clear bools
    velocity_set = false;
    calculation_done = false;
    busy = false;
    ramp_started = false;

    positionIsReady = false;
    orientationIsReady =false;

    //Regler
//    KP[0] = 0.08;   KI[0] = 0.0;   KD[0] = 0.000;    windUP[0] = MAX_JNT_VELOCITIES[0];
//    KP[1] = 0.1;    KI[1] = 0.0;   KD[0] = 0.000;    windUP[1] = MAX_JNT_VELOCITIES[1];
//    KP[2] = 0.07;   KI[2] = 0.0;  KD[0] = 0.000;    windUP[2] = MAX_JNT_VELOCITIES[2];
//    KP[3] = 0.125;  KI[3] = 0.0;    KD[0] = 0.000;    windUP[3] = MAX_JNT_VELOCITIES[3];
//    KP[4] = 0.1;    KI[4] = 0.0;    KD[0] = 0.000;    windUP[4] = MAX_JNT_VELOCITIES[4];

    KP[0] = 0.1;    KI[0] = 0.0;   KD[0] = 0.000;    windUP[0] = MAX_JNT_VELOCITIES[0];
    KP[1] = 0.1;    KI[1] = 0.0;   KD[0] = 0.000;    windUP[1] = MAX_JNT_VELOCITIES[1];
    KP[2] = 0.1;    KI[2] = 0.0;   KD[0] = 0.000;    windUP[2] = MAX_JNT_VELOCITIES[2];
    KP[3] = 0.1;    KI[3] = 0.0;   KD[0] = 0.000;    windUP[3] = MAX_JNT_VELOCITIES[3];
    KP[4] = 0.1;    KI[4] = 0.0;   KD[0] = 0.000;    windUP[4] = MAX_JNT_VELOCITIES[4];

//    GAIN = 5.0;
    fehlerQ = 0.0;
    emax = 0.0;
    emin = 0.0;

    //Iterators
    points = 0;
    timeIt = 0;
    gripperIt = 0;
    pPoints = 0;

    //clear vectors
    pathParam.clear();
    gripperPoint.clear();
    gripperPosition.clear();
    maxParamValue.clear();
    DiffPath.clear();
    coefficientsSegment.clear();
    coefficients_ppoint.clear();
    timeIncPoint.clear();
    maxJointParamValue.clear();
    pointParam.clear();
    degreeOfFreedom.clear();
    Diff.clear();

    maxJointParamValue.resize(ARMJOINTS);
    degreeOfFreedom.resize(ARMJOINTS);
    pointParam.resize(ARMJOINTS);
    Diff.resize(ARMJOINTS);

    timeIncBase.clear();
    timeBaseIt = 0;
    startBasePose = lastBasePose;

    baseCoeffs.clear();
}

void motionPlanning::newTrajectory(JointPosition current_position)
{
    init();
    setcurrentJointPosition(current_position);
}

void motionPlanning::update(JointPosition current_position)
{
    current_position.subtractOffset();

    ros::Time timeNow = ros::Time::now();
    double passedTime = (timeNow - startTime).toSec();
    double delta_t = (timeNow - lastUpdateTime).toSec();
    lastUpdateTime = timeNow;
    unsigned long index = (unsigned long)(timeIncPoint.size() * passedTime / totalTime);

//    ROS_INFO("index %d/%d", (int)index, (int)timeIncPoint.size());
//    ROS_INFO("time %f/%f", passedTime, totalTime);

    for (size_t k = 0; k < ARMJOINTS; k++)
    {
        if (index < timeIncPoint.size())
        {
            armJointVelocities.velocities[k].value = timeIncPoint[index].arm_axis[k].velocity;
            armJointPositions.positions[k].value = timeIncPoint[index].arm_axis[k].position;
            armJointAccelerations.accelerations[k].value = timeIncPoint[index].arm_axis[k].acceleration;
        }

        else
        {
            armJointPositions.positions[k].value = timeIncPoint.back().arm_axis[k].position;
            armJointVelocities.velocities[k].value = 0;
            armJointAccelerations.accelerations[k].value = 0;
        }

        /*---------------------------------------------Regler--------------------------------------*/

        // Regelabweichung bestimmen
        ePos[k] = armJointPositions.positions[k].value - current_position[k];
        ePosSum[k] = ePos[k] + ePosSum[k];
        armJointVelocities.velocities[k].value = armJointVelocities.velocities[k].value +
                KP[k]*(ePos[k] + KI[k]*ePosSum[k]*delta_t +  KD[k]*(ePos[k] - ePosAlt[k])/delta_t);

        // Anti Wind-Up
        if (armJointVelocities.velocities[k].value > windUP[k])
            armJointVelocities.velocities[k].value = windUP[k];
        if (armJointVelocities.velocities[k].value < (-1)*windUP[k])
            armJointVelocities.velocities[k].value = (-1)*windUP[k];

        ePosAlt[k] = ePos[k];


        /*---------------------------------------------------------------    ----------------------*/

    }

    // this prevents that the goal tolerance is checked right at the beginning of the movement
    // which would cause problems if start point is close to end point.
    if(passedTime/totalTime < tolerance_check_percentage)
        return;

    // check goal tolerance
    int jointCounter = 0;
    for (size_t k = 0; k < ARMJOINTS; k++)
    {
        double goal_diff = fabs(timeIncPoint.back().arm_axis[k].position - current_position[k]);

        if(goal_diff < goal_tolerance)
        {
            jointCounter++;
        }
    }

    if (jointCounter >= 5)
    {
        // Done
        busy = false;
    }

    if (index >= timeIncPoint.size())
    {        
            // Done
            busy = false;
    }
}

JointPosition motionPlanning::getPosition()
{
    JointPosition pos;
    for(uint i=0; i< armJointPositions.positions.size(); i++)
    {
        pos[i] = armJointPositions.positions[i].value;
    }

    pos.addOffset();

    return pos;
}
JointVelocity motionPlanning::getVelocity()
{
    JointVelocity vel;
    for(uint i=0; i< armJointVelocities.velocities.size(); i++)
    {
        vel[i] = armJointVelocities.velocities[i].value;
    }

    return vel;
}
JointAcceleration motionPlanning::getAcceleration()
{
    JointAcceleration acc;
    for(uint i=0; i< armJointAccelerations.accelerations.size(); i++)
    {
        acc[i] = armJointAccelerations.accelerations[i].value;
    }

    return acc;
}

void motionPlanning::setcurrentJointPosition(JointPosition current_position)
{
    current_position.subtractOffset();

    for (size_t k = 0; k < JOINTSPACE; k++)
        pointParam[k].position = current_position[k];

    pathParam.push_back(pointParam);
}


int motionPlanning::setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5)
{
    // Prüfen ob der Bahnpunkt im Abeitsraum liegt
//    if(checkPositionRange(JointPos1, JointPos2, JointPos3, JointPos4, JointPos5) == 0)
//        return 0;

    // Wenn Geschwidigkeit für diesen Punkt nicht gesetzt ist, automatisch setzten
    if (!velocity_set)
        setTargetJointVelocity(1.0);

    // Gelenkewinkel für den Bahnpunkt setzen
    pointParam[0].position = JointPos1;
    pointParam[1].position = JointPos2;
    pointParam[2].position = JointPos3;
    pointParam[3].position = JointPos4;
    pointParam[4].position = JointPos5;

    // Bahnpunkt in Liste hinzufügen
    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;

    return 1;
}

int motionPlanning::setPathPoint(std::vector <double> JointPos)
{
    // Prüfen ob der Bahnpunkt im Abeitsraum liegt

//    if(checkPositionRange(JointPos[0], JointPos[1], JointPos[2], JointPos[3], JointPos[4]) == 0)
//        return 0;

    // Wenn Geschwidigkeit für diesen Punkt nicht gesetzt ist, automatisch setzten
    if (!velocity_set)
        setTargetJointVelocity(1.0);

    // Gelenkewinkel für den Bahnpunkt setzen + Gripperposition
    for (size_t i = 0; i < JOINTSPACE; i++)
        pointParam[i].position = JointPos[i];

    if (JointPos.size() > JOINTSPACE) {
        gripperPosition.push_back(JointPos.back());
        gripperPoint.push_back(points);
    }
    // Bahnpunkt in Liste hinzufügen
    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;

    return 1;
}

int motionPlanning::setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5, double gripperPos)
{
    // Prüfen ob der Bahnpunkt im Abeitsraum liegt
//    if(checkPositionRange(JointPos1, JointPos2, JointPos3, JointPos4, JointPos5) == 0)
//        return 0;

    // Wenn Geschwidigkeit für diesen Punkt nicht gesetzt ist, automatisch setzten
    if (!velocity_set)
        setTargetJointVelocity(1.0);

    // Gelenkewinkel für den Bahnpunkt setzen + Gripperposition
    pointParam[0].position = JointPos1;
    pointParam[1].position = JointPos2;
    pointParam[2].position = JointPos3;
    pointParam[3].position = JointPos4;
    pointParam[4].position = JointPos5;
    gripperPosition.push_back(gripperPos);
    gripperPoint.push_back(points);
    // Bahnpunkt in Liste hinzufügen
    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;

    return 1;
}

int motionPlanning::setPathPoint(double JointPos[5])
{
    // Prüfen ob der Bahnpunkt im Abeitsraum liegt
//    if(checkPositionRange(JointPos[0], JointPos[1], JointPos[2], JointPos[3], JointPos[4]) == 0)
//        return 0;

    // Wenn Geschwidigkeit für diesen Punkt nicht gesetzt ist, automatisch setzten
    if (!velocity_set)
        setTargetJointVelocity(1.0);

    // Gelenkewinkel für den Bahnpunkt setzen
    for (size_t k = 0; k < ARMJOINTS; k++) {
        pointParam[k].position = JointPos[k];
    }

    // Bahnpunkt in Liste hinzufügen
    pathParam.push_back(pointParam);
    points++;
    velocity_set = false;

    return 1;
}

int motionPlanning::checkPositionRange(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5)
{
    // Prüfe, ob Winkel im Arbeitsbereich vo Gelenk 1 liegt
    if ((JointPos1 < initParamValues[JOINT1].minAngle) || (JointPos1 > initParamValues[JOINT1].maxAngle)) {
        ROS_ERROR("Jointposition 1 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos1);
        return 0;
    }
    // Prüfe, ob Winkel im Arbeitsbereich vo Gelenk 1 liegt
    else if ((JointPos2 < initParamValues[JOINT2].minAngle) || (JointPos2 > initParamValues[JOINT2].maxAngle)) {
        ROS_ERROR("Jointposition 2 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos2);
        return 0;
    }
    // Prüfe, ob Winkel im Arbeitsbereich vo Gelenk 1 liegt
    else if ((JointPos3 < initParamValues[JOINT3].minAngle) || (JointPos3 > initParamValues[JOINT3].maxAngle)) {
        ROS_ERROR("Jointposition 3 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos3);
        return 0;
    }
    // Prüfe, ob Winkel im Arbeitsbereich vo Gelenk 1 liegt
    else if ((JointPos4 < initParamValues[JOINT4].minAngle) || (JointPos4 > initParamValues[JOINT4].maxAngle)) {
        ROS_ERROR("Jointposition 4 of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos4);
        return 0;
    }
    // Prüfe, ob Winkel im Arbeitsbereich vo Gelenk 1 liegt
    else if ((JointPos5 < initParamValues[JOINT5].minAngle) || (JointPos5 > initParamValues[JOINT5].maxAngle)) {
        ROS_ERROR("Jointposition 5  of Pathpoint %i is out of Range: %.3f.", int(points+1), JointPos5);
        return 0;
    }

    return 1;
}

void motionPlanning::setTargetJointVelocity(double JointVel1, double JointVel2, double JointVel3, double JointVel4, double JointVel5)
{

    // Geschwindigkeit für Gelenk 1 setzen
    if (JointVel1 < initParamValues[JOINT1].max_vel) {
        maxJointParamValue[JOINT1].max_vel = JointVel1;
        maxJointParamValue[JOINT1].max_acc = initParamValues[JOINT1].max_acc;
    }

    // Geschwindigkeit für Gelenk 2 setzen
    if (JointVel2 < initParamValues[JOINT2].max_vel) {
        maxJointParamValue[JOINT2].max_vel = JointVel2;
        maxJointParamValue[JOINT2].max_acc = initParamValues[JOINT2].max_acc;
    }

    // Geschwindigkeit für Gelenk 3 setzen
    if (JointVel3 < initParamValues[JOINT3].max_vel) {
        maxJointParamValue[JOINT3].max_vel = JointVel3;
        maxJointParamValue[JOINT3].max_acc = initParamValues[JOINT3].max_acc;
    }

    // Geschwindigkeit für Gelenk 4 setzen
    if (JointVel4 < initParamValues[JOINT4].max_vel) {
        maxJointParamValue[JOINT4].max_vel = JointVel4;
        maxJointParamValue[JOINT4].max_acc = initParamValues[JOINT4].max_acc;
    }

    // Geschwindigkeit für Gelenk 5 setzen
    if (JointVel5 < initParamValues[JOINT5].max_vel) {
        maxJointParamValue[JOINT5].max_vel = JointVel5;
        maxJointParamValue[JOINT5].max_acc = initParamValues[JOINT5].max_acc;
    }

    // Geschwindigkeite zur Liste hinzufügen
    maxParamValue.push_back(maxJointParamValue);
    velocity_set = true;
}

void motionPlanning::setTargetJointVelocity(vector <double> JointVel)
{

    // Prüfen, ob Anzahl der Parameter übereinstimmt
    if (JointVel.size() != ARMJOINTS)
        ROS_ERROR("setTargetJointVelocity expects 5 Velocities. %d are given", (int)JointVel.size());

    // Geschwindigkeiten für Gelenk 1-5 setzen
    for (size_t k = 0; k < ARMJOINTS; k++)
    {
        if (JointVel[k] < initParamValues[k].max_vel)
            maxJointParamValue[k].max_vel = JointVel[k];
        else
            maxJointParamValue[k].max_vel = initParamValues[k].max_vel;
        maxJointParamValue[k].max_acc = initParamValues[k].max_acc;
    }

    // Geschwindigkeite zur Liste hinzufügen
    maxParamValue.push_back(maxJointParamValue);
    velocity_set = true;
}

void motionPlanning::setTargetJointVelocity(double velfactor)
{
    if (velfactor < 0.0) {
        ROS_ERROR("Velocity musst be bigger than 0");
        velfactor = 0.1;
    }

    for (size_t k = 0; k < ARMJOINTS; k++)
    {
        maxJointParamValue[k].max_vel = initParamValues[k].max_vel*velfactor;
        maxJointParamValue[k].max_acc = initParamValues[k].max_acc;
    }

    // Geschwindigkeite zur Liste hinzufügen
    maxParamValue.push_back(maxJointParamValue);
    velocity_set = true;
}


void motionPlanning::calculatePathProperties()
{
    double t_vmax = 0.0;
    double t_amax = 0.0;
    double t_max = 0.0;
    double t_f = 0.0;
    COEFFS coefficientsJoint;

    //Winkeldifferenz zweier Bahnpunkte bestimmen --> Diff = q_z - q-s
    for (size_t l = 0; l < points; l++) {
        for (size_t k = 0; k < ARMJOINTS; k++) {
            Diff[k] = pathParam[l+1][k].position - pathParam[l][k].position;
        }
        DiffPath.push_back(Diff);
    }

    //Berechnen der Maximalen Trajektorierenzeit auf Basis der Maximalen Geschwindikeiten/Beschleunigungen
    for (size_t l = 0; l < points; l++) {
        for (size_t k = 0; k < JOINTSPACE; k++) {
            t_vmax = 15.0/8.0*fabs(DiffPath[l][k])/maxParamValue[l][k].max_vel;
            t_amax = sqrt(10.0/sqrt(3.0)*fabs(DiffPath[l][k])/maxParamValue[l][k].max_acc);
            t_max = max(t_vmax, t_amax);
            if (t_max > t_f)
                t_f = t_max;
        }
        t_f_ppoint.push_back(t_f);
        t_f = 0.0;
    }

    //Geschwindigkeiten und Beschleunigungen  an den Bahnpunkten setzen
    for (size_t l = 0; l <= points; l++)	{
        for (size_t k = 0; k < JOINTSPACE; k++) {
            if(l == 0) {
                pathParam[l][k].acceleration = 0.0;
                pathParam[l][k].velocity = 0.0;
            }
            else if(l == points) {
                pathParam[l][k].velocity = 0.0;
                pathParam[l][k].acceleration = 0.0;
            }
            else {
                if(pathParam[l-1][k].position > pathParam[l][k].position &&
                   pathParam[l][k].position < pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = 0.0;
                    pathParam[l][k].acceleration = sqrt(10/sqrt(3)*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1]);
                }
                else if(pathParam[l-1][k].position > pathParam[l][k].position &&
                        pathParam[l][k].position > pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = -10/8*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1];
                    pathParam[l][k].acceleration = 0.0;
                }
                else if(pathParam[l-1][k].position < pathParam[l][k].position &&
                        pathParam[l][k].position < pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = 10/8*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1];
                    pathParam[l][k].acceleration = 0.0;
                }
                else if(pathParam[l-1][k].position < pathParam[l][k].position &&
                        pathParam[l][k].position > pathParam[l+1][k].position)
                {
                    pathParam[l][k].velocity = 0.0;
                    pathParam[l][k].acceleration = -sqrt(10/sqrt(3)*fabs(DiffPath[l-1][k])/t_f_ppoint[l-1]);
                }
                else
                {
                    pathParam[l][k].velocity = 0.0;
                    pathParam[l][k].acceleration = 0.0;
                }
            }
        }
    }

    //Bestimmung der Koeffizienten
    for (size_t l = 0; l < points; l++) {
        for (size_t k = 0; k < JOINTSPACE; k++) {
            setPolynomCoefficients(pathParam[l][k], pathParam[l+1][k], t_f_ppoint[l], coefficientsJoint);
            coefficientsSegment.push_back(coefficientsJoint);
        }
        // Koeeffizienten zur Liste hinzufügen
        coefficients_ppoint.push_back(coefficientsSegment);
        // Koeffizienten fü temporäres Bahnsegment löschen
        coefficientsSegment.clear();
    }
    calculation_done = true;
}

void motionPlanning::setPolynomCoefficients(PARAM _paramInit, PARAM _paramGoal, double _trajecTime, COEFFS &_coefficient)
{

    // Berechne Koeffizient a0
    _coefficient.a0 = _paramInit.position;
    // Berechne Koeffizient a1
    _coefficient.a1 = _paramInit.velocity;
    // Berechne Koeffizient a2
    _coefficient.a2 = _paramInit.acceleration/2;
    // Berechne Koeffizient a3
    _coefficient.a3 = (20*(_paramGoal.position - _paramInit.position) -
                       (8*_paramGoal.velocity + 12*_paramInit.velocity)*_trajecTime -
                       (3*_paramInit.acceleration - _paramGoal.acceleration)*pow(_trajecTime,2)) / (2*pow(_trajecTime,3));
    // Berechne Koeffizient a4
    _coefficient.a4 = (30*(_paramInit.position - _paramGoal.position) +
                       (14*_paramGoal.velocity + 16*_paramInit.velocity)*_trajecTime +
                       (3*_paramInit.acceleration - 2*_paramGoal.acceleration)*pow(_trajecTime,2)) / (2*pow(_trajecTime,4));
    // Berechne Koeffizient a5
    _coefficient.a5 = (12*(_paramGoal.position - _paramInit.position) -
                       6*(_paramGoal.velocity + _paramInit.velocity)*_trajecTime -
                       (_paramInit.acceleration - _paramGoal.acceleration)*pow(_trajecTime,2)) / (2*pow(_trajecTime,5));

}

void motionPlanning::offlineMotionPlanning()
{

    // Timer definieren
    double wrappedTime;
    gripperIt = 0;
    totalTime = 0;

    // Randbedingungen entsprechend der Bahnvorgabe setzen
    calculatePathProperties();

    for (size_t l = 0; l < points; l++)
    {
        // Timer auf Null setzen
        wrappedTime = 0.0;
        // Gripper Position setzen
        if (!gripperPoint.empty())
        {
            if (gripperPoint[gripperIt] == l)
            {
                gripperPoint[gripperIt] = timeIncPoint.size();
                gripperIt++;
            }
        }

        boolParam.push_back(true);

        // Gelenkwinkel setzen für jeden diskreten Zeitpunkt
        while (wrappedTime < t_f_ppoint[l])
        {
            for (size_t k = 0; k < JOINTSPACE; k++)
            {
                // Boolparam füllen
                if (wrappedTime != 0)
                    boolParam.push_back(false);

                // Interpolierten Sollwert für diskreten Zeitpunkt berechnen für...
                setPolynomFunction(coefficients_ppoint[l][k], wrappedTime, degreeOfFreedom[k]);
                // ...Position
                singleAxis.arm_axis[k].position = degreeOfFreedom[k].position;
                // ...Geschwindigkeit
                singleAxis.arm_axis[k].velocity = degreeOfFreedom[k].velocity;
                // ...Beschleunigung
                singleAxis.arm_axis[k].acceleration = degreeOfFreedom[k].acceleration;
            }

            // Zeitschritt inkrementieren
            wrappedTime = wrappedTime + LOOPRATE / 10; // 10-fache Auflösung, um bei Frequenzschwankungen kein Sprünge zu haben
            // Interpolierte Werte in Vektor ablegen
            timeIncPoint.push_back(singleAxis);
        }
        totalTime += wrappedTime;
    }
    gripperIt = 0;
    ROS_INFO("Berechnung abgeschlossen");
}

void motionPlanning::setPolynomFunction(COEFFS _coeff, double _time, PARAM &_param)
{

    // Position für diskreten Zeitschritt berechnen
    _param.position = 1*_coeff.a0 + 1*_coeff.a1*_time + 1*_coeff.a2*pow(_time,2) + 1*_coeff.a3*pow(_time,3) +
                      1*_coeff.a4*pow(_time,4) + 1*_coeff.a5*pow(_time,5);

    // Geschwindigkeit für diskreten Zeitschritt berechnen
    _param.velocity = 1*_coeff.a1 + 2*_coeff.a2*_time + 3*_coeff.a3*pow(_time,2) + 4*_coeff.a4*pow(_time,3) +
                      5*_coeff.a5*pow(_time,4);

    // Beschleunigung für diskreten Zeitschritt berechnen
    _param.acceleration = 2*_coeff.a2 + 6*_coeff.a3*_time + 12*_coeff.a4*pow(_time,2) + 20*_coeff.a5*pow(_time,3);
}

void motionPlanning::moveToPose()
{
    if(calculation_done)
    {
        // Startzeit merken
        startTime = ros::Time::now();
        lastUpdateTime = startTime;

        // Arm für weitere Bewegungen blocken
        busy = true;
    }
    else
    {
        // Wenn Trajektorie nicht generiert ist, Anwendung schließen und Fehler ausgeben
        ROS_ERROR("Calculation not ready!");
    }
}


bool motionPlanning::isBusy() {
    return busy;
}


luh_youbot_kinematics::JointPosition motionPlanning::getPositionError()
{
    JointPosition pos_error;
    for(uint i=0; i<pos_error.size(); i++)
    {
        pos_error[i] = ePos[i];
    }

    return pos_error;
}

void motionPlanning::stop()
{
    busy = false;
}
