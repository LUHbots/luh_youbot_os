/* *****************************************************************
 *
 * luh_youbot_gui
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

#include "luh_youbot_gui/main_window.hpp"

using namespace luh_youbot_gui;

//########## CONSTRUCTOR ###############################################################################################
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent)
{
    // main window
    setWindowTitle("LUH Youbot Manipulator Interface");
    setWindowIcon(QIcon(":/images/icon.png"));

    // metatypes for signals
    qRegisterMetaType<luh_youbot_kinematics::JointPosition>("luh_youbot_kinematics::JointPosition");
    qRegisterMetaType<luh_youbot_kinematics::CylindricPosition>("luh_youbot_kinematics::CylindricPosition");
    qRegisterMetaType<luh_youbot_kinematics::CartesianPosition>("luh_youbot_kinematics::CartesianPosition");
    qRegisterMetaType<luh_youbot_kinematics::JointVelocity>("luh_youbot_kinematics::JointVelocity");
    qRegisterMetaType<luh_youbot_kinematics::CylindricVelocity>("luh_youbot_kinematics::CylindricVelocity");
    qRegisterMetaType<luh_youbot_kinematics::CartesianVelocity>("luh_youbot_kinematics::CartesianVelocity");

    // status bar
    status_bar_ = new QStatusBar();
    this->setStatusBar(status_bar_);
    status_bar_->showMessage("Main Window initialised.");

    // arm control window (central widget)
    arm_control_window_ = new ArmControlWindow();
    connect(arm_control_window_, SIGNAL(logMessage(QString)), this, SLOT(showMessage(QString)));
    connect(arm_control_window_, SIGNAL(cartesianVelocity(double,double,double)),
            &qnode_, SLOT(setCartesianVelocity(double,double,double)));
    connect(arm_control_window_, SIGNAL(jointVelocity(int,double)), &qnode_, SLOT(setJointVelocity(int,double)));
    connect(arm_control_window_, SIGNAL(jointPositionSet(luh_youbot_kinematics::JointPosition,double,int,bool)),
            &qnode_, SLOT(setJointPosition(luh_youbot_kinematics::JointPosition,double,int,bool)));
    connect(arm_control_window_, SIGNAL(cartesianPositionSet(luh_youbot_kinematics::CartesianPosition,double,int,bool)),
            &qnode_, SLOT(setCartesianPosition(luh_youbot_kinematics::CartesianPosition,double,int,bool)));
    connect(arm_control_window_, SIGNAL(cylindricPositionSet(luh_youbot_kinematics::CylindricPosition,double,int,bool)),
            &qnode_, SLOT(setCylindricPosition(luh_youbot_kinematics::CylindricPosition,double,int,bool)));
    connect(arm_control_window_->render_area()->model(), SIGNAL(gripperPositionCommand(double)),
            &qnode_, SLOT(setGripperMsg(double)));
    this->setCentralWidget(arm_control_window_);
    connect(arm_control_window_, SIGNAL(compensateGravityRequested(bool)), &qnode_, SLOT(compensateGravity(bool)));
//    connect(arm_control_window_, SIGNAL(stiffenArmRequested()), &qnode_, SLOT(stiffenArm()));
    connect(arm_control_window_, SIGNAL(velocitiesChanged(luh_youbot_kinematics::JointVelocity,
                                                          luh_youbot_kinematics::CartesianVelocity,
                                                          luh_youbot_kinematics::CylindricVelocity)),
            &qnode_, SLOT(changeVelocities(luh_youbot_kinematics::JointVelocity,
                                          luh_youbot_kinematics::CartesianVelocity,
                                          luh_youbot_kinematics::CylindricVelocity)));
    // QNode
    connect(&qnode_, SIGNAL(logMessage(QString)), this, SLOT(showMessage(QString)));
    connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));
//    connect(&qnode_, SIGNAL(jointStateUpdated(luh_youbot_kinematics::JointPosition,double)),
//            arm_control_window_, SLOT(onCurrentJointstateChanged(luh_youbot_kinematics::JointPosition,double)));
    connect(&qnode_, SIGNAL(jointStateUpdated(double,double,double,double,double,double)),
            arm_control_window_, SLOT(onCurrentJointstateChanged(double,double,double,double,double,double)));
    connect(&qnode_, SIGNAL(isInitialised()), arm_control_window_, SLOT(onRosInitialised()));

    qnode_.init(argc, argv);

}

//########## SLOT: SHOW MESSAGE ########################################################################################
void MainWindow::showMessage(QString msg)
{
    status_bar_->showMessage(msg);
}

//########## CLOSE EVENT ###############################################################################################
void MainWindow::closeEvent(QCloseEvent *event)
{
    qnode_.cancel();

    QMainWindow::closeEvent(event);
}
