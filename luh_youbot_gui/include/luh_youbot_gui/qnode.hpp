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
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/

#ifndef LUH_YOUBOT_GUI__QNODE_HPP_
#define LUH_YOUBOT_GUI__QNODE_HPP_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <luh_youbot_msgs/MoveToJointPoseAction.h>
#include <luh_youbot_msgs/MoveToCylindricPoseAction.h>
#include <luh_youbot_msgs/MoveToCartesianPoseAction.h>
#include <luh_youbot_msgs/SetGripperAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <QString>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float32.h>

#include "common.hpp"


namespace luh_youbot_gui
{

typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveToJointPoseAction> JointPoseClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveToCylindricPoseAction> CylindricPoseClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveToCartesianPoseAction> CartesianPoseClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::SetGripperAction> GripperClient;

class QNode : public QThread {
    Q_OBJECT

    enum ActionClientType
    {
        JOINT_DIRECT,
        JOINT_INTER,
        JOINT_PLAN,
        CARTESIAN_DIRECT,
        CARTESIAN_INTER,
        CARTESIAN_PLAN,
        CYLINDRIC_DIRECT,
        CYLINDRIC_INTER,
        CYLINDRIC_PLAN,
        NONE
    };

public:
    QNode();
	virtual ~QNode();
    bool init(int argc, char** argv);
	void run();
    void cancel(){cancel_is_requested_ = true;}

Q_SIGNALS:
    void rosShutdown();
    void logMessage(QString msg);
//    void jointStateUpdated(luh_youbot_kinematics::JointPosition, double gripper_state);
    void jointStateUpdated(double q1, double q2, double q3, double q4, double q5, double gripper_state);
    void isInitialised();
    void actionDone();

public Q_SLOTS:
    void setCartesianVelocity(double v_x, double v_y, double v_theta);
    void setJointVelocity(int joint_idx, double vel);
    void setJointPosition(luh_youbot_kinematics::JointPosition pos, double gripper, int mode, bool is_relative);
    void setCylindricPosition(luh_youbot_kinematics::CylindricPosition pos, double gripper, int mode, bool is_relative);
    void setCartesianPosition(luh_youbot_kinematics::CartesianPosition pos, double gripper, int mode, bool is_relative);
    void setGripperAction(double width);
    void setGripperMsg(double width);
    void compensateGravity(bool do_compensation);
//    void stiffenArm();
    void changeVelocities(luh_youbot_kinematics::JointVelocity joint_vel,
                          luh_youbot_kinematics::CartesianVelocity cart_vel,
                          luh_youbot_kinematics::CylindricVelocity cyl_vel);

private:

    ros::Timer timer_;
    ros::Subscriber joint_state_subscriber_;
    ros::Publisher cylindric_velocity_publisher_;
    ros::Publisher joint_velocity_publisher_;

    JointPoseClient *joint_pose_client_direct_;
    JointPoseClient *joint_pose_client_inter_;
    JointPoseClient *joint_pose_client_plan_;

    CylindricPoseClient *cylindric_pose_client_direct_;
    CylindricPoseClient *cylindric_pose_client_inter_;
    CylindricPoseClient *cylindric_pose_client_plan_;

    CartesianPoseClient *cartesian_pose_client_direct_;
    CartesianPoseClient *cartesian_pose_client_inter_;
    CartesianPoseClient *cartesian_pose_client_plan_;

    GripperClient *gripper_client_;

    ros::ServiceClient compensate_gravity_client_;
    ros::ServiceClient deactivate_gravity_comp_client_;
//    ros::ServiceClient stiffen_client_;

    ros::ServiceClient set_cart_vel_client_;
    ros::ServiceClient set_cyl_vel_client_;
    ros::ServiceClient set_jnt_vel_client_;

    ros::Publisher gripper_command_publisher_;
    ros::Time last_gripper_pub_time_;

    bool is_initialised_;
    int jointstate_downsampling_factor_;
    int jointstate_count_;

    double positioning_timeout_;

    ActionClientType active_client_;

    bool cancel_is_requested_;

    double gripper_timeout_;
    double gripper_setpoint_;

    std_msgs::Float32 gravity_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state);
    void jointActionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const luh_youbot_msgs::MoveToJointPoseResultConstPtr& result);
    void cartesianActionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                     const luh_youbot_msgs::MoveToCartesianPoseResultConstPtr& result);
    void cylindricActionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                     const luh_youbot_msgs::MoveToCylindricPoseResultConstPtr& result);
    void timerCallback(const ros::TimerEvent &evt);

};

}  // namespace luh_youbot_gui

#endif /* luh_youbot_gui_QNODE_HPP_ */
