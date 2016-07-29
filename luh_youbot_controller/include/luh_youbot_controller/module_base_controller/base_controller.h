/* *****************************************************************
 *
 * luh_youbot_controller
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

#ifndef LUH_YOUBOT_CONTROLLER_MODULE_BASE_CONTROLLER_H
#define LUH_YOUBOT_CONTROLLER_MODULE_BASE_CONTROLLER_H

#include "../module_base_class/controller_module.h"
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <luh_youbot_msgs/MoveBaseAction.h>
#include <luh_youbot_msgs/AlignBaseToPoseAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <numeric>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <std_srvs/Empty.h>
#include <luh_laser_watchdog/Distances.h>
#include <luh_youbot_msgs/ApproachBaseAction.h>
#include <luh_youbot_msgs/GetBasePose.h>

class ModuleBaseController : public ControllerModule
{
public:

    typedef actionlib::SimpleActionServer<luh_youbot_msgs::MoveBaseAction> MoveBaseServer;
    typedef actionlib::SimpleActionServer<luh_youbot_msgs::AlignBaseToPoseAction> AlignBaseServer;
    typedef actionlib::SimpleActionServer<luh_youbot_msgs::ApproachBaseAction> ApproachServer;
    typedef luh_youbot_msgs::MoveBaseGoal BasePose;
    typedef luh_youbot_msgs::AlignBaseToPoseGoal AlignGoal;
    typedef luh_youbot_msgs::ApproachBaseGoal ApproachGoal;
    ModuleBaseController();
    ~ModuleBaseController();

protected:

    AlignBaseServer* align_base_server_;
    MoveBaseServer* move_base_server_;
    ApproachServer* approach_server_;
    ros::Subscriber velocity_subscriber_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber laser_subscriber_;
    ros::ServiceServer stop_server_;
    ros::ServiceServer get_pose_server_;

    ros::Publisher error_publisher_;

    BasePose start_pose_;
    BasePose goal_pose_;
    BasePose current_pose_;

    ros::Time last_update_time_;

    AlignGoal align_goal_;
    std::queue<BasePose> last_two_errors_;
    std::vector<double> time_memory_;
    std::vector<BasePose> error_memory_;
    int memory_index_;
    BasePose accumulated_error_;

    ApproachGoal approach_goal_;
    luh_laser_watchdog::Distances distances_;

    geometry_msgs::Twist velocity_command_;
    luh_youbot_msgs::MoveBaseResult moved_distance_;
    double max_velocity_x_;
    double max_velocity_y_;
    double max_velocity_theta_;

    double max_velocity_x_approach_;
    double max_velocity_y_approach_;
    double max_velocity_theta_approach_;

    double velocity_p_factor_x_;
    double velocity_p_factor_y_;
    double velocity_p_factor_theta_;
    double velocity_i_factor_x_;
    double velocity_i_factor_y_;
    double velocity_i_factor_theta_;
    double velocity_d_factor_x_;
    double velocity_d_factor_y_;
    double velocity_d_factor_theta_;
    double position_tolerance_x_;
    double position_tolerance_y_;
    double position_tolerance_theta_;
    double velocity_command_timeout_;
    int memory_size_;
    double align_stop_timeout_;
    double align_fail_timeout_;
    std::string align_topic_;

    tf::StampedTransform align_pose_transform_;
    bool got_align_transform_;

    double min_dist_x_;
    double min_dist_y_;

    int print_counter_;

//    ros::Time time_memory_;

    enum ActionMode
    {
        POSITION,
        VELOCITY,
        ALIGN,
        APPROACH,
        IDLE
    }mode_;

    void init();
    void activate();
    void deactivate();
    void update();
    void emergencyStop();
    void preempt();

    void updateVelocityMode();
    void updatePositionMode();
    void updateAlignMode();
    void updateApproachMode();
    void updateMovedDistance();

    void approachCallback();
    void moveBaseCallback();
    void alignBaseCallback();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void velocityCallback(const geometry_msgs::Twist::ConstPtr &velocity_msg);
    void laserCallback(const luh_laser_watchdog::Distances::ConstPtr &distances);
    void preemptCallback();
    bool stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool getPoseCallback(luh_youbot_msgs::GetBasePose::Request &req, luh_youbot_msgs::GetBasePose::Response &res);

    bool activated_;


};

#endif // MODULE_BASE_CONTROLLER_H
