/* *****************************************************************
 *
 * luh_youbot_manipulation
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

#ifndef LUH_YOUBOT_MANIPULATION_MODULE_JOINT_TRAJECTORY_H
#define LUH_YOUBOT_MANIPULATION_MODULE_JOINT_TRAJECTORY_H

#include "luh_youbot_manipulation/module_base_class/manipulation_module.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

class ModuleJointTrajectory : public ManipulationModule
{
public:

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryActionServer;

    ModuleJointTrajectory();
    ~ModuleJointTrajectory();

protected:

    struct TrajectoryPoint
    {
        luh_youbot_kinematics::JointPosition position;
        ros::Duration time;
    };
    typedef std::vector<TrajectoryPoint> Trajectory;

    void init();
    void activate();
    void deactivate();
    void update();    
    void emergencyStop();

    bool activated_;

    TrajectoryActionServer *server_;
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;

    Trajectory trajectory_;
    ros::Time start_time_;
    bool has_active_goal_;
    bool trajectory_ended_;
    Trajectory::iterator last_point_, next_point_;
    luh_youbot_kinematics::JointPosition actual_position_;
    luh_youbot_kinematics::JointPosition desired_position_;
    luh_youbot_kinematics::JointPosition position_error_;

    luh_youbot_kinematics::JointPosition path_tolerance_;
    luh_youbot_kinematics::JointPosition default_path_tolerance_;
    luh_youbot_kinematics::JointPosition goal_tolerance_;
    luh_youbot_kinematics::JointPosition default_goal_tolerance_;
    double goal_time_tolerance_;

    void goalCallback();
};

#endif // LUH_YOUBOT_MANIPULATION_MODULE_JOINT_TRAJECTORY_H
