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


#ifndef LUH_YOUBOT_MANIPULATION_MODULE_GRAVITY_COMPENSATION_H
#define LUH_YOUBOT_MANIPULATION_MODULE_GRAVITY_COMPENSATION_H

#include "../module_base_class/manipulation_module.h"
#include <luh_youbot_msgs/GetLoad.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <luh_youbot_kinematics/arm_dynamics.h>
#include <deque>
#include <luh_youbot_msgs/ForceFitAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<luh_youbot_msgs::ForceFitAction> ForceFitServer;

class ModuleGravityCompensation : public ManipulationModule
{
public:
    ModuleGravityCompensation();
    ~ModuleGravityCompensation();

    void init();
    void activate();
    void deactivate();
    void update();
    void emergencyStop();

protected:

    luh_youbot_kinematics::StaticParameters params_;

    ForceFitServer* force_fit_server_;

    bool is_active_;
    bool do_compensation_;

    ros::ServiceServer compensate_server_;
    ros::ServiceServer deactivate_server_;
    ros::ServiceServer get_load_server_;

    luh_youbot_kinematics::YoubotArmDynamics dynamics_;

    int buffer_size_;

    // force fitting
    double force_;
    double displacement_;
    double start_r_;
    double start_z_;

    std::deque<luh_youbot_kinematics::JointPosition> position_buffer_;
    std::deque<luh_youbot_kinematics::JointEffort> torque_buffer_;

    bool compensateCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool getLoadCallback(luh_youbot_msgs::GetLoad::Request &req, luh_youbot_msgs::GetLoad::Response &res);
    bool deactivateCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void forceFitCallback();
    void preemptCallback();

    luh_youbot_kinematics::JointEffort getEffortFromForce(const luh_youbot_kinematics::JointPosition &pos);
    bool checkGoal(const luh_youbot_kinematics::JointPosition &pos);
};

#endif // MODULE_GRAVITY_COMPENSATION_H
