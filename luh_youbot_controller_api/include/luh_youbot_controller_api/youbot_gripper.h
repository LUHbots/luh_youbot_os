/* *****************************************************************
 *
 * luh_youbot_controller_api
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
 * @brief  Contains the YoubotGripper class.
 */

#ifndef LUH_YOUBOT_CONTROLLER_API_GRIPPER_H
#define LUH_YOUBOT_CONTROLLER_API_GRIPPER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <luh_youbot_msgs/MoveToCartesianPoseAction.h>
#include <luh_youbot_msgs/MoveToCylindricPoseAction.h>
#include <luh_youbot_msgs/MoveToJointPoseAction.h>
#include <luh_youbot_msgs/MoveToNamedPoseAction.h>
#include <luh_youbot_msgs/SetGripperAction.h>
#include <luh_youbot_msgs/GripObjectAction.h>
#include <luh_youbot_kinematics/arm_kinematics.h>

namespace youbot_api
{

typedef actionlib::SimpleActionClient<luh_youbot_msgs::GripObjectAction> GripObjectClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::SetGripperAction> SetGripperClient;

/**
 * @brief This is a wrapper class for all action clients that control the youBot gripper.
 * @details Example usage:
 * @code
 * // initialise ROS
 * ros::init(argc, argv, "example node");
 * ros::NodeHandle node_handle;
 *
 * // initialise gripper
 * youbot_api::YoubotGripper gripper;
 * gripper.init(node_handle);
 *
 * // open gripper
 * gripper.open();
 * gripper.waitForCurrentAction();
 *
 * // close gripper
 * gripper.setWidth(0);
 *
 * @endcode
 */
class YoubotGripper
{
public:
    /**
     * @brief YoubotGripper constructor. Does not initialise the action clients.
     * @details The object must be initialised with the init() method before it can be used.
     * @see YoubotGripper#init.
     */
    YoubotGripper();
    ~YoubotGripper();

    /**
     * @brief Initialises the action clients.
     * @param node Node handle.
     */
    void init(ros::NodeHandle &node);

    /**
     * @brief Grips a known object.
     * @details The object has to be listed in the configuration file of the gripper module in luh_youbot_controller.
     * @param object_name Object name.
     */
    void gripObject(std::string object_name);

    /**
     * @brief Opens or closes the gripper to the specified width.
     * @param value Desired gripper width in meters.
     * @param is_relative Specifies if the value is relative to current width.
     */
    void setWidth(double value, bool is_relative=false);

    /**
     * @brief Opens the gripper to the maximum width.
     */
    void open();

    /**
     * @brief Checks if the gripper is busy.
     * @return True if the gripper is busy.
     */
    bool isBusy(){return is_busy_;}

    /**
     * @brief Waits until the current action is finished.
     * @param timeout Optional timeout in seconds after which the action will be aborted.
     * @return
     */
    bool waitForCurrentAction(double timeout=0);

    /**
     * @brief Aborts the currently executed action.
     */
    void abortCurrentAction();

    /**
     * @brief Checks if the last action has finished successfully.
     * @return True if last action has succeeded.
     */
    bool actionSucceeded();

protected:
    enum ActiveClient
    {
        GRIP_OBJECT,
        SET_GRIPPER
    } active_client_;

    ros::NodeHandle *node_;

    SetGripperClient *set_gripper_client_;
    GripObjectClient *grip_object_client_;

    bool is_busy_;

    void onSetGripperActionDone(const actionlib::SimpleClientGoalState& state,
                                const luh_youbot_msgs::SetGripperResultConstPtr& result);
    void onGripObjectActionDone(const actionlib::SimpleClientGoalState& state,
                                const luh_youbot_msgs::GripObjectResultConstPtr& result);



};

}

#endif // LUH_YOUBOT_CONTROLLER_API_GRIPPER_H
