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

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/luh_youbot_gui/qnode.hpp"
#include <luh_youbot_msgs/CylindricVector.h>
#include <luh_youbot_msgs/JointVector.h>
#include <std_srvs/Empty.h>
#include <luh_youbot_msgs/SetCartesianVelocity.h>
#include <luh_youbot_msgs/SetCylindricVelocity.h>
#include <luh_youbot_msgs/SetJointVelocity.h>

namespace luh_youbot_gui
{

//########## CONSTRUCTOR ###############################################################################################
QNode::QNode():
    is_initialised_(false),
    cancel_is_requested_(false)
{

}

//########## DESTRUCTOR ################################################################################################
QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

//########## INIT ######################################################################################################
bool QNode::init(int argc, char **argv)
{
    ros::init(argc, argv, "luh_youbot_gui");

    start();
    return true;
}

//########## RUN #######################################################################################################
void QNode::run()
{
    while ( ! ros::master::check() && ros::ok())
    {
        Q_EMIT logMessage("Waiting for ROS master...");
        this->sleep(1);
        if(cancel_is_requested_)
        {
            this->quit();
            return;
        }
    }

    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // === PARAMETERS ===
    jointstate_downsampling_factor_ = 10;//TODO: from config
    jointstate_count_ = 0;
    positioning_timeout_ = 10;
    active_client_ = NONE;
    gripper_setpoint_ = 0.0;
    gripper_timeout_ = 3.0;

    // === ROS COMMUNICATION ===
    joint_state_subscriber_ = n.subscribe("/arm_1/joint_states", 10, &QNode::jointStateCallback, this);
    cylindric_velocity_publisher_ = n.advertise<luh_youbot_msgs::CylindricVector>("arm_1/cylindric_velocity", 1);
    joint_velocity_publisher_ = n.advertise<luh_youbot_msgs::JointVector>("arm_1/joint_velocity", 1);

    joint_pose_client_direct_ = new JointPoseClient("/arm_1/to_joint_pose/direct", true);
    joint_pose_client_inter_ = new JointPoseClient("/arm_1/to_joint_pose/inter", true);
    joint_pose_client_plan_ = new JointPoseClient("/arm_1/to_joint_pose/plan", true);

    cylindric_pose_client_direct_ = new CylindricPoseClient("/arm_1/to_cylindric_pose/direct", true);
    cylindric_pose_client_inter_ = new CylindricPoseClient("/arm_1/to_cylindric_pose/inter", true);
    cylindric_pose_client_plan_ = new CylindricPoseClient("/arm_1/to_cylindric_pose/plan", true);

    cartesian_pose_client_direct_ = new CartesianPoseClient("/arm_1/to_cartesian_pose/direct", true);
    cartesian_pose_client_inter_ = new CartesianPoseClient("/arm_1/to_cartesian_pose/inter", true);
    cartesian_pose_client_plan_ = new CartesianPoseClient("/arm_1/to_cartesian_pose/plan", true);

    gripper_client_ = new GripperClient("/arm_1/set_gripper", true);

    gripper_command_publisher_ = n.advertise<std_msgs::Float32>("arm_1/gripper_command", 1);

    compensate_gravity_client_ = n.serviceClient<std_srvs::Empty>("arm_1/compensate_gravity");
    deactivate_gravity_comp_client_ = n.serviceClient<std_srvs::Empty>("arm_1/deactivate_gravity_compensation");
//    stiffen_client_ = n.serviceClient<std_srvs::Empty>("arm_1/stiffen");
    set_cart_vel_client_ = n.serviceClient<luh_youbot_msgs::SetCartesianVelocity>("arm_1/set_cartesian_velocity");
    set_cyl_vel_client_ = n.serviceClient<luh_youbot_msgs::SetCylindricVelocity>("arm_1/set_cylindric_velocity");
    set_jnt_vel_client_ = n.serviceClient<luh_youbot_msgs::SetJointVelocity>("arm_1/set_joint_velocity");

    timer_ = n.createTimer(ros::Duration(positioning_timeout_), &QNode::timerCallback, this, true, false);

    while(!joint_pose_client_direct_->waitForServer(ros::Duration(1)))
    {
        Q_EMIT logMessage("Waiting for luh_youbot_manipulation node...");

        if(!ros::ok())
        {
            Q_EMIT rosShutdown();
            this->quit();
            return;
        }

        if(cancel_is_requested_)
        {
            ros::shutdown();
            this->quit();
            return;
        }
    }

    while(joint_state_subscriber_.getNumPublishers() == 0)
    {
        Q_EMIT logMessage("Waiting for jointstate publisher...");
        ros::Duration(1).sleep();

        if(!ros::ok())
        {
            Q_EMIT rosShutdown();
            this->quit();
            return;
        }

        if(cancel_is_requested_)
        {
            ros::shutdown();
            this->quit();
            return;
        }
    }

    is_initialised_ = true;

    Q_EMIT isInitialised();

    Q_EMIT logMessage("Ros is spinning...");

    ros::spin();

    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

//########## CALLBACK: TIMER ###########################################################################################
void QNode::timerCallback(const ros::TimerEvent &evt)
{
    timer_.stop();

    switch(active_client_)
    {
    case JOINT_DIRECT:
        joint_pose_client_direct_->cancelAllGoals();
        break;
    case JOINT_INTER:
        joint_pose_client_inter_->cancelAllGoals();
        break;
    case JOINT_PLAN:
        joint_pose_client_plan_->cancelAllGoals();
        break;
    case CYLINDRIC_DIRECT:
        cylindric_pose_client_direct_->cancelAllGoals();
        break;
    case CYLINDRIC_INTER:
        cylindric_pose_client_inter_->cancelAllGoals();
        break;
    case CYLINDRIC_PLAN:
        cylindric_pose_client_plan_->cancelAllGoals();
        break;
    case CARTESIAN_DIRECT:
        cartesian_pose_client_direct_->cancelAllGoals();
        break;
    case CARTESIAN_INTER:
        cartesian_pose_client_inter_->cancelAllGoals();
        break;
    case CARTESIAN_PLAN:
        cartesian_pose_client_plan_->cancelAllGoals();
        break;
    case NONE:
        return;
    }

    Q_EMIT logMessage("Action timed out.");

    active_client_ = NONE;

}

//########## CALLBACK: JOINTSTATE ######################################################################################
void QNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state)
{
    if(jointstate_count_ == jointstate_downsampling_factor_)
        jointstate_count_ = 0;
    else
    {
        jointstate_count_++;
        return;
    }

    luh_youbot_kinematics::JointPosition joint_position;
    joint_position.setValues(joint_state->position);
    joint_position.addOffset();

    //    ROS_INFO("jointstate received.");
    //    joint_position.printValues("incoming jointstate");

    double gripper_state = 0.0;
    if(joint_state->position.size() >= 7)
    {
        gripper_state = joint_state->position[5] + joint_state->position[6];
        // TODO: use gripper kinematics
    }

    Q_EMIT jointStateUpdated(joint_position.q1(),
                             joint_position.q2(),
                             joint_position.q3(),
                             joint_position.q4(),
                             joint_position.q5(),
                             gripper_state);
}

//########## SLOT: CARTESIAN VELOCITY ##################################################################################
void QNode::setCartesianVelocity(double v_x, double v_y, double v_theta)
{
    if(!is_initialised_)
        return;

    luh_youbot_msgs::CylindricVector v;

    v.r = v_x;
    v.z = v_y;
    v.theta = v_theta;

    cylindric_velocity_publisher_.publish(v);
}

//########## SLOT: JOINT VELOCITY ######################################################################################
void QNode::setJointVelocity(int joint_idx, double vel)
{
    if(!is_initialised_)
        return;

    luh_youbot_msgs::JointVector v;

    if(joint_idx == 0)
        v.q1 = vel;
    else if(joint_idx == 1)
        v.q2 = vel;
    else if(joint_idx == 2)
        v.q3 = vel;
    else if(joint_idx == 3)
        v.q4 = vel;
    else if(joint_idx == 4)
        v.q5 = vel;

    // DEBUG:
    //    Q_EMIT logMessage("joint 2 velocity: " + QString::number(v.q2));
    //    std::cout << "joint 2 velocity: " << v.q2 << std::endl;

    joint_velocity_publisher_.publish(v);
}

//########## SLOT: JOINT POSITION ######################################################################################
void QNode::setJointPosition(luh_youbot_kinematics::JointPosition pos, double gripper, int mode, bool is_relative)
{
    if(active_client_ != NONE)
        return;

    JointPoseClient* client;

    if(mode == MODE_DIRECT)
    {
        client = joint_pose_client_direct_;
        active_client_ = JOINT_DIRECT;
    }
    else if(mode == MODE_INTER)
    {
        client = joint_pose_client_inter_;
        active_client_ = JOINT_INTER;
    }
    else
    {
        client = joint_pose_client_plan_;
        active_client_ = JOINT_PLAN;
    }

    luh_youbot_msgs::MoveToJointPoseGoal goal;
    goal.pose_is_relative = is_relative;
    goal.pose.q1 = pos.q1();
    goal.pose.q2 = pos.q2();
    goal.pose.q3 = pos.q3();
    goal.pose.q4 = pos.q4();
    goal.pose.q5 = pos.q5();

    Q_EMIT logMessage("Arm is moving...");

    client->sendGoal(goal,
                     boost::bind(&QNode::jointActionDoneCallback, this, _1, _2),
                     JointPoseClient::SimpleActiveCallback(),
                     JointPoseClient::SimpleFeedbackCallback());

    gripper_setpoint_ = gripper;

    timer_.start();
}

//########## SLOT: CYLINDRIC POSITION ##################################################################################
void QNode::setCylindricPosition(luh_youbot_kinematics::CylindricPosition pos, double gripper, int mode, bool is_relative)
{
    if(active_client_ != NONE)
        return;

    CylindricPoseClient* client;

    if(mode == MODE_DIRECT)
    {
        client = cylindric_pose_client_direct_;
        active_client_ = CYLINDRIC_DIRECT;
    }
    else if(mode == MODE_INTER)
    {
        client = cylindric_pose_client_inter_;
        active_client_ = CYLINDRIC_INTER;
    }
    else
    {
        client = cylindric_pose_client_plan_;
        active_client_ = CYLINDRIC_PLAN;
    }

    luh_youbot_msgs::MoveToCylindricPoseGoal goal;
    goal.pose_is_relative = is_relative;
    goal.pose.q1 = pos.q1();
    goal.pose.r = pos.r();
    goal.pose.z = pos.z();
    goal.pose.theta = pos.theta();
    goal.pose.q5 = pos.q5();

    Q_EMIT logMessage("Arm is moving...");

    client->sendGoal(goal,
                     boost::bind(&QNode::cylindricActionDoneCallback, this, _1, _2),
                     CylindricPoseClient::SimpleActiveCallback(),
                     CylindricPoseClient::SimpleFeedbackCallback());

    gripper_setpoint_ = gripper;

    timer_.start();
}

//########## SLOT: CARTESIAN POSITION ##################################################################################
void QNode::setCartesianPosition(luh_youbot_kinematics::CartesianPosition pos, double gripper, int mode, bool is_relative)
{
    if(active_client_ != NONE)
        return;

    CartesianPoseClient* client;

    if(mode == MODE_DIRECT)
    {
        client = cartesian_pose_client_direct_;
        active_client_ = CARTESIAN_DIRECT;
    }
    else if(mode == MODE_INTER)
    {
        client = cartesian_pose_client_inter_;
        active_client_ = CARTESIAN_INTER;
    }
    else
    {
        client = cartesian_pose_client_plan_;
        active_client_ = CARTESIAN_PLAN;
    }

    luh_youbot_msgs::MoveToCartesianPoseGoal goal;
    goal.pose_is_relative = is_relative;
    goal.pose.x = pos.x();
    goal.pose.y = pos.y();
    goal.pose.z = pos.z();
    goal.pose.theta = pos.theta();
    goal.pose.q5 = pos.q5();
    goal.pose.header.frame_id = "arm_link_0";

    Q_EMIT logMessage("Arm is moving...");

    client->sendGoal(goal,
                     boost::bind(&QNode::cartesianActionDoneCallback, this, _1, _2),
                     CartesianPoseClient::SimpleActiveCallback(),
                     CartesianPoseClient::SimpleFeedbackCallback());

    gripper_setpoint_ = gripper;

    timer_.start();
}

//########## CALLBACK: JOINT ACTION DONE ###############################################################################
void QNode::jointActionDoneCallback(const actionlib::SimpleClientGoalState& state,
                             const luh_youbot_msgs::MoveToJointPoseResultConstPtr& result)
{
    timer_.stop();

    active_client_ = NONE;

    if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        Q_EMIT logMessage("Move to pose action failed.");
    }
    else
    {
        Q_EMIT logMessage("Goal pose reached.");
    }

    setGripperAction(gripper_setpoint_);
}

//########## CALLBACK: CARTESIAN ACTION DONE ###########################################################################
void QNode::cartesianActionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const luh_youbot_msgs::MoveToCartesianPoseResultConstPtr& result)
{
    timer_.stop();

    active_client_ = NONE;

    if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        Q_EMIT logMessage("Move to pose action failed.");
    }
    else
    {
        Q_EMIT logMessage("Goal pose reached.");
    }

    setGripperAction(gripper_setpoint_);
}

//########## CALLBACK: CYLINDRIC ACTION DONE ###########################################################################
void QNode::cylindricActionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const luh_youbot_msgs::MoveToCylindricPoseResultConstPtr& result)
{
    timer_.stop();

    active_client_ = NONE;

    if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        Q_EMIT logMessage("Move to pose action failed.");
    }
    else
    {
        Q_EMIT logMessage("Goal pose reached.");

    }

    setGripperAction(gripper_setpoint_);
}

//########## SET GRIPPER (ACTION) ######################################################################################
void QNode::setGripperAction(double width)
{
    luh_youbot_msgs::SetGripperGoal goal;
    goal.gripper_width = width;

    gripper_client_->sendGoal(goal);

    if(!gripper_client_->waitForResult(ros::Duration(gripper_timeout_)))
    {
        Q_EMIT logMessage("Gripper action timed out.");
    }
    else if(gripper_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        Q_EMIT logMessage("Gripper action failed.");
    }

    Q_EMIT actionDone();
}

//########## SET GRIPPER (MESSAGE) #####################################################################################
void QNode::setGripperMsg(double width)
{
    double delta_t = (ros::Time::now() - last_gripper_pub_time_).toSec();
    if(delta_t < 0.1)
        return;
    std_msgs::Float32 msg;
    msg.data = width;
    gripper_command_publisher_.publish(msg);
    last_gripper_pub_time_ = ros::Time::now();
}

//########## COMPENSATE GRAVITY ########################################################################################
void QNode::compensateGravity(bool do_compensation)
{
    std_srvs::Empty srv;
    if(do_compensation)
    {
        compensate_gravity_client_.call(srv);
        Q_EMIT logMessage("Gravity compensation activated.");
    }
    else
    {
        deactivate_gravity_comp_client_.call(srv);
        Q_EMIT logMessage("Gravity compensation deactivated.");
    }
}

//########## STIFFEN ARM ###############################################################################################
//void QNode::stiffenArm()
//{
//    std_srvs::Empty srv;
//    stiffen_client_.call(srv);
//}

//########## CHANGE VELOCITIES #########################################################################################
void QNode::changeVelocities(luh_youbot_kinematics::JointVelocity joint_vel,
                      luh_youbot_kinematics::CartesianVelocity cart_vel,
                      luh_youbot_kinematics::CylindricVelocity cyl_vel)
{
    luh_youbot_msgs::SetCartesianVelocity cart_srv;
    cart_srv.request.max_velocity.x = cart_vel.x();
    cart_srv.request.max_velocity.y = cart_vel.y();
    cart_srv.request.max_velocity.z = cart_vel.z();
    cart_srv.request.max_velocity.theta = cart_vel.theta();
    cart_srv.request.max_velocity.q5 = cart_vel.q5();

    if(!set_cart_vel_client_.call(cart_srv))
    {
        Q_EMIT logMessage("Failed to set cartesian velocity.");
    }

    luh_youbot_msgs::SetCylindricVelocity cyl_srv;
    cyl_srv.request.max_velocity.q1 = cyl_vel.q1();
    cyl_srv.request.max_velocity.r = cyl_vel.r();
    cyl_srv.request.max_velocity.z = cyl_vel.z();
    cyl_srv.request.max_velocity.theta = cyl_vel.theta();
    cyl_srv.request.max_velocity.q5 = cyl_vel.q5();

    if(!set_cyl_vel_client_.call(cyl_srv))
    {
            Q_EMIT logMessage("Failed to set cylindrical velocity.");
    }

    luh_youbot_msgs::SetJointVelocity jnt_srv;
    jnt_srv.request.max_velocity.q1 = joint_vel.q1();
    jnt_srv.request.max_velocity.q2 = joint_vel.q2();
    jnt_srv.request.max_velocity.q3 = joint_vel.q3();
    jnt_srv.request.max_velocity.q4 = joint_vel.q4();
    jnt_srv.request.max_velocity.q5 = joint_vel.q5();

    if(!set_jnt_vel_client_.call(jnt_srv))
    {
        Q_EMIT logMessage("Failed to set joint velocity.");
    }

}

}  // namespace luh_youbot_gui
