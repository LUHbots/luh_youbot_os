/* *****************************************************************
 *
 * youbot_motion_control
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
 * @file   motionPlanning.hpp
 * @author Eduard Popp <eduardpopp@web.de>
 * @date   28. März 2013
 *
 * @brief  Diese Datei enthält die Klasse motionPlanning, die diverse Methoden zur Bahnplanung bereithält
 *
 * Diese Klasse dient zur Plannung einer Trajektorie mithilfe eines Polynoms 5.Ordnung im Jointspace und eingeschränkt auch im Taskspace.
 * Dadurch sind flüssige und ruckfreie Bewegungen möglich. Vor der Plannung findet eine Synchronisation die langsamste Achse statt, sodass
 * alle Gelenkwinkel gleichzeitig beginnen und stoppen. Mit dieser Klasse ist eine PTP- als auch eine kontinuirliche Bewegung ohne Anhalten
 * an Bahnpunkten möglich, was im Folgenden als "überschleifen" bezeichnet wird. Die Geschwindigkeit lässt sich ebenfalls für jedes Segment
 * variabel regulieren.
 *
 * NEUERUNGEN:
 * Geschwindigkeitssetter können ausgelassen werden: dazu velpoints eingeführt, die die
 * Anzahl an Bahnpunkten, mit der Anzahl an Geschwindigkeitspunkten vor jedem Setzen eines
 * neuen Bahnpunkts vergleichen. Wenn ungleich, wird Standardgeschwindigkeit gesetzt(velfactor=1).
 *
 * moveToPose kann auch nach einem Pathpoint gesetzt werden. Wird fälschicherweise die Methode aufgerufen,
 * wird geprüft, ob ein Endpunkt bereits gesetzt wurde, indem eine boolsche Variable(endPose_set) geprüft wird.
 * Die Variable wird nur dann true gesetzt, wenn ein Endpunkt aufgerufen wurde. *
 */
/* Modified by Simon Aden 2014 */

#pragma once

#ifndef LUH_YOUBOT_CONTROLLER_MOTION_PLANNING_HPP
#define LUH_YOUBOT_CONTROLLER_MOTION_PLANNING_HPP

//ROS includes
#include "ros/ros.h"
#include "rosbag/recorder.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//C++ includes
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

//YouBot includes
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <luh_youbot_driver_api/youbot_interface.h>
#include <luh_youbot_kinematics/arm_kinematics.h>

class motionPlanning {

public:


    /**
     *@brief constructor der motionPlanning Klasse
     */
    motionPlanning();

    /**
     *@brief destructor der motionPlanning Klasse
     */
    ~motionPlanning();


    /**
     * @brief Initialisiert alle Variablen und Vektoren, die für neue Berechnung benötigt werden
     * @param Parameter legt die Freiheitsgrade fest.
     */
    void init();


    luh_youbot_kinematics::JointPosition getPositionError();

    /**
     * @brief Methode ruft die Initialisierung für eine neue Bahnplanung im Jointspace auf.
     */
    void newTrajectory(luh_youbot_kinematics::JointPosition current_position);

    /**
      *@brief setzt Softwareentschalter für jeden Joint und prüft, ob diese Überschritten sind.
      *       Bei Überschreitung werden die betroffenen Joints über ein Geschwindigkeitsprpofil zu stehen gebracht.
      */    
    void checkJointLimit();

    /**
      *@brief Für die Berechnung des Bahnprofils wird ein Polynom 5.Ordnung verwendet. In dieser Methode werden zunächst die Trajektorienzeiten für jeden
      *       Gelenkwinkel berechnet und die weitere Berechnung auf den langsamsten Arm synchronisiert, indem die maximalen Geschindigkeiten/bzw. Beschleunigungen
      *       der einzelnen Gelenkwinkel angepasst werden.
      *       Bei einer PTP-Bewegung sind Ziel- und Start Beschleunigungen, sowie Geschwindigkeiten 0. Bei einem Überschliffen Punkt ist das Positionsbedingt:
      *       Für P(i-1) > P(i) > P(i+1): v = -vmax , a = 0;
      *       Für P(i-1) > P(i) < P(i+1): v = 0     , a = amax;
      *       Für P(i-1) < P(i) > P(i+1): v = 0     , a = -amax;
      *       Für P(i-1) < P(i) < P(i+1): v = vmax  , a = 0;
      *       Im letzten Schritt erfolgt mithilfe der Randbedingungen die Berechnung der Polynomkoeffizienten.
      */    
    void calculatePathProperties();

    /**
      *@brief setze absolute Geschwindigkeiten der Gelenkwinkel für das Bahnsegment. Geschwindigkeit kann nicht größer als max. Geschwindigkeit sein.
      *@param JointVel1 absolute Geschwindigkeit für den 1.Gelenkwinkel
      *@param JointVel2 absolute Geschwindigkeit für den 2.Gelenkwinkel
      *@param JointVel3 absolute Geschwindigkeit für den 3.Gelenkwinkel
      *@param JointVel4 absolute Geschwindigkeit für den 4.Gelenkwinkel
      *@param JointVel5 absolute Geschwindigkeit für den 5.Gelenkwinkel
      */    
    void setTargetJointVelocity(double JointVel1, double JointVel2, double JointVel3, double JointVel4, double JointVel5);

    /**
      *@brief setze absolute Geschwindigkeiten der Gelenkwinkel für das Bahnsegment. Geschwindigkeit kann nicht größer als max. Geschwindigkeit sein.
      *@param JointVel Vektor mit 5 Elementen für die Gelenkwinkel
      */    
    void setTargetJointVelocity(std::vector <double> JointVel);

    /**
      *@brief setze Geschwindigkeit relativ zur max. Geschwindigkeit.
      *@param velfactor Faktor, der mit den Maximalgeschwindigkeiten multipliziert wird. Faktor muss größer als 0 sein.
      */    
    void setTargetJointVelocity(double velfactor);

    /**
      *@brief setze einen Bahnpunkt mit überschleifen
      *@param JointPos1 Zielgelenkwinkel der 1.Achse
      *@param JointPos1 Zielgelenkwinkel der 2.Achse
      *@param JointPos1 Zielgelenkwinkel der 3.Achse
      *@param JointPos1 Zielgelenkwinkel der 4.Achse
      *@param JointPos1 Zielgelenkwinkel der 5.Achse
      *@param gripper Gripperposition publishen, wenn true
      */    
    int setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5);

    int setPathPoint(std::vector <double> JointPos);

    int setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5, double gripperPos);

    /**
      *@brief setze einen Bahnpunkt mit überschleifen
      *@param JointPos Vektor mit 5 Elementen für Zielgelenkwinkelpositionen
      *@param gripper Gripperposition publishen, wenn true
      */    
    int setPathPoint(double JointPos[5]);

    /**
      *@brief gibt true wieder, wenn der noch in Bewegung ist, sonst false
      *@return true bedeutet, dass der Arm busy ist.
      */
    bool isBusy();

    /**
      *@brief Nach dem setzen der Geschwindigkeiten und Bahnpunkte wird mit Aufruf dieser Funktion die Berechnung ausgeführt
      */
    void moveToPose();

    /**
      *@brief Generierung von Bahnpunkten entlang einer Strecke zwischen Ziel und Endpunkt im Kartesischen Raum.
      *       Die Bahnpunkte werden über die IK in Gelenkwinkelkoordinaten umgerechnet und in einer neuen Bahnplannung im Gelenkwinkelraum überschliffen.
      */
    bool offlineMotionPlanning();

    /**
      *@brief Update Funktion, die die Jointstates aus 'youbot' bezieht, und keine Messages publisht
      */
    void update(luh_youbot_kinematics::JointPosition current_position);

    luh_youbot_kinematics::JointPosition getPosition();
    luh_youbot_kinematics::JointVelocity getVelocity();
    luh_youbot_kinematics::JointAcceleration getAcceleration();


    /**
     * @brief Set frequency of update function.
     * @param loop_rate Frequency of update function in 1/s
     */
    void setUpdateFrequency(double frequency);

    /**
     * @brief Set tolerance for goal position.
     * @param tolerance Joint angle tolerance in radians.
     * @param check_percentage Goal tolerance is only checked if at least this percentage of the trajectory is covered.
     */
    void setGoalTolerance(double tolerance, double check_percentage){goal_tolerance = tolerance;
                                                                     tolerance_check_percentage = check_percentage;}

    /**
     * @brief Stop current arm movement.
     */
    void stop();

    /// Wenn Arm in Bewegung, ist die Variable true
    bool busy;

    /// Reglerparameter und -auswertung
    double GAIN;
    std::vector <double> windUP;
    std::vector <double> stellwert;
    std::vector <double> KP;
    std::vector <double> KI;
    std::vector <double> KD;
    double fehlerQ, emax, emin;

private:
    /// Struktur mit maximaler Geschwindigeit max_vel und maximaler Beschleunigung max_acc
    struct JointProperties {
        double max_acc;
        double max_vel;
        double minAngle;
        double maxAngle;
    } JointProp;

    /// Struktur die eine Bahnpunkt beschreibt durch Position, Geschwindigkeit und Beschleunigung
    struct parameter {
        double position;
        double velocity;
        double acceleration;
    };

    struct axis {
        std::vector <parameter> arm_axis;
    };

    /// Struktur die 6 Koeffizienten beinhaltet wodurch ein Polynom 5.Ordnung ausgedrückt wird
    struct coefficients {
        double a0;
        double a1;
        double a2;
        double a3;
        double a4;
        double a5;
    };

    typedef struct parameter PARAM;
    typedef struct coefficients COEFFS;
    typedef struct JointProperties PROPERTIES;
    typedef struct axis AXIS;

    /**
     * @brief setPolynomCoefficients
     * @param _paramInit
     * @param _paramGoal
     * @param _trajecTime
     * @param _coefficient
     */
    void setPolynomCoefficients(PARAM _paramInit, PARAM _paramGoal, double _trajecTime, COEFFS &_coefficient);


    /**
     * @brief setPolynomFunction
     * @param _coeff
     * @param _time
     * @param _param
     */
    void setPolynomFunction(COEFFS _coeff, double _time, PARAM &_param);


    /**
      *@brief Setze die Ist-Position als ersten Bahn/-Starpunkt
      */
    void setcurrentJointPosition(luh_youbot_kinematics::JointPosition current_position);




    /**
     * @brief checkPositionRange
     * @param JointPos1
     * @param JointPos2
     * @param JointPos3
     * @param JointPos4
     * @param JointPos5
     * @return 1 = alles in Ordnung, 0 = Fehler bei der Berechnung
     */
//    int checkPositionRange(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5);


//    ros::NodeHandle n;
    std::string configFile;

    size_t orientation;

    geometry_msgs::Pose	lastBasePose;
    geometry_msgs::Pose	startBasePose;
    geometry_msgs::Pose firstBasePose;
    geometry_msgs::Pose secondBasePose;

    double periode;
    double jnt[5];
    tf::Quaternion quatRot;

    std::vector <double> gripperPosition;
    std::vector <size_t> gripperPoint;
    //std::vector <brics_actuator::JointValue> gripperJointPositions;

    /// Messagetyp der Joinstates
    //brics_actuator::JointVelocities currentVelocities;
    //brics_actuator::JointVelocities armJointVelocities;
    //brics_actuator::JointAccelerations armJointAccelerations;
    //brics_actuator::JointPositions armJointPositions;
    //luh_youbot_kinematics::JointVelocity currentVelocities;
    luh_youbot_kinematics::JointVector armJointVelocities;
    luh_youbot_kinematics::JointAcceleration armJointAccelerations;
    luh_youbot_kinematics::JointPosition armJointPositions;
    //brics_actuator::JointPositions gripperPositionCommand;
    std_msgs::Bool armJointBool;
    geometry_msgs::Twist baseVelocity;    
    //brics_actuator::JointPositions currentPositions;
    //brics_actuator::JointPositions regler;
    //luh_youbot_kinematics::JointPosition currentPositions;
    //luh_youbot_kinematics::JointPosition regler;

    geometry_msgs::Twist cmd_vel;

    /// Ziel und Startwerte im Taskspace (für CartBahnplanung benötigt)
    double *xEndeffector;

    COEFFS pitchCoeff, rollCoeff, jntTwoCoeff, timeRel;
    PARAM jntTwoParamInit, jntTwoParamGoal;
    AXIS singleAxis, base;

    /// Vektor mit absoluten maximalen Trajektorienzeiten aller Bahnsegmente
    std::vector<double> t_f_ppoint;
    std::vector <double> t_f_base;

    size_t timeIt;
    size_t gripperIt;
    size_t timeBaseIt;

    double totalTime;
    ros::Time startTime;
    ros::Time lastUpdateTime;


    /// Dieser Vektor beinhaltet die Gelenkwinkeldifferenz zweier aufeinanderfolgender Bahnpunkte aller Gelenkwinkel.
    std::vector <double> Diff;

    /// Vektor in Vektor mit den Gelenkwinkeldifferenzen aller Bahnpunkte einer Bahn.
    std::vector <std::vector<double> > DiffPath;

    /// Vektor beinhaltet Position, Geschwindigkeit und Beschleunigung zu jedem inkrementellen Zeitpunkt
    std::vector <struct axis> timeIncPoint;
    std::vector <struct axis> timeIncBase;
    /// Vektor beinhaltet Geschwindigkeits- und Beschleunigungsinformation aller Joints eines Bahnpunkts
    std::vector<struct JointProperties> maxJointParamValue;

    /// Vektor beinhaltet Geschwindigkeits- und Beschleunigungsinformation aller Bahnpunkte
    std::vector <std::vector <PROPERTIES> > maxParamValue;

    /// Vektor mit maximalen Beschleunigungs- und Geschwindigkeitswerten für jeden Joint
    //std::vector<PROPERTIES> initParamValues;

    /// Vektor mit Beschreibung eines Bahnpunkts für jeden Joint
    std::vector< PARAM > pointParam;

    std::vector<PARAM> degreeOfFreedom;

    std::vector<bool> boolParam;

    /// Vektor in Vektor mit Beschreibung aller Bahnpunkte
    std::vector <std::vector <PARAM> > pathParam;
    PARAM baseX, baseY, basePhi;
    std::vector <PARAM> baseParam;

    /// Vektor, der die Polynomkoeffizienten a0...a5 eines Bahnsegments beinhaltet
    std::vector<COEFFS> coefficientsSegment;

    /// Vektor in Vektor, der die Polynomkoeffizienten a0...a5 aller Bahnsegmente beinhaltet
    std::vector <std::vector<COEFFS> > coefficients_ppoint;
    std::vector <COEFFS> baseCoeffs;

    /// At the beginning true. If Joint is out of range set false, so starting time is set only once
    bool ramp_started;
    bool positionIsReady;
    bool orientationIsReady;

    /// true if velocity is bigger than 0.1
    bool positiv;

    /// true if velocity ist lower than -0.1
    bool negativ;

    bool velocity_set;
    bool calculation_done;

    bool positionRange;
    bool onlyOrientation;

    size_t points;
    size_t pPoints;

    /// Positionsabweichung
    double *ePos, e;
    double *ePosSum, eSum;
    double *ePosAlt, eAlt;

    /// Time when the Jointboundaries are reached
    double ramp_start_time;
    double start_time;
    struct timeval detail_time;
    struct timeval new_time;
    struct timeval detail_ramp;

    /// Loop rate
    double LOOPRATE;

    /// Konstanten
    static const unsigned int JOINT1  = 0;
    static const unsigned int JOINT2  = 1;
    static const unsigned int JOINT3  = 2;
    static const unsigned int JOINT4  = 3;
    static const unsigned int JOINT5  = 4;
    static const unsigned int GRIPPER_L = 0;
    static const unsigned int GRIPPER_R = 1;
    //static const unsigned int ARMJOINTS = 5;
    static const unsigned int TESTJOINT = 0;

//    static const double MINANGLE1 = 0.576 * M_PI/180;
//    static const double MINANGLE2 = 0.576 * M_PI/180;
//    static const double MINANGLE3 = -288.0 * M_PI/180;
//    static const double MINANGLE4 = 1.266 * M_PI/180;
//    static const double MINANGLE5 = 6.337 * M_PI/180;
//    static const double MINANGLEGRIPPER = 0.000 * M_PI/180;

//    static const double MAXANGLE1 = 334.616 * M_PI/180;
//    static const double MAXANGLE2 = 150.0 * M_PI/180;
//    static const double MAXANGLE3 = -0.9 * M_PI/180;
//    static const double MAXANGLE4 = 196.5 * M_PI/180;
//    static const double MAXANGLE5 = 323.241 * M_PI/180;
//    static const double MAXPOSGRIPPER = 0.060 * M_PI/180;

    /*static const double MINANGLE1 = -2.9395372288 + 2.949606436;
    static const double MINANGLE2 = -1.124398163 + 1.1344673701987218;
    static const double MINANGLE3 =  -2.4783710285 - 2.5481772172532176;
    static const double MINANGLE4 = -1.7668361332 + 1.7889600250839740;
    static const double MINANGLE5 = -2.8913271881 + 3.0019466477485340;
    static const double MINANGLEGRIPPER = 0.000 * M_PI/180;

    static const double MAXANGLE1 = 2.8905337534 + 2.949606436;
    static const double MAXANGLE2 = 1.4835265078 + 1.1344673701987218;
    static const double MAXANGLE3 =  2.532469254 - 2.5481772172532176;
    static const double MAXANGLE4 = 1.6402432236 + 1.7889600250839740;
    static const double MAXANGLE5 = 2.6396457936 + 3.0019466477485340;
    static const double MAXPOSGRIPPER = 0.060 * M_PI/180;
*/

//    static const double VMAX1 = 1.3;
//    static const double VMAX2 = 1.3;
//    static const double VMAX3 = 1.3;
//    static const double VMAX4 = 1.5;
//    static const double VMAX5 = 1.5;

//    static const double AMAX1 = 1.0;//1.343;
//    static const double AMAX2 = 1.25;//1.343;
//    static const double AMAX3 = 1.25;//2.094;
//    static const double AMAX4 = 2.0;
//    static const double AMAX5 = 2.0;

    static const double BASE_VMAX_X = 0.5;
    static const double BASE_VMAX_Y = 0.5;
    static const double BASE_VMAX_PHI = 0.5;

//    static const double LOOPRATE = 0.002;

    static const size_t JOINTSPACE = 5;
    static const size_t TRANSLATION = 3;
    static const size_t ORIENTATION = 3;



    /// Publisher für die Geschwindigkeit an das cmd_velocity Topic Base
    struct velocity{
        double velX;
        double velY;
    };


    double goal_tolerance;

    double tolerance_check_percentage;
};

#endif // MOTION_PLANNING_HPP
