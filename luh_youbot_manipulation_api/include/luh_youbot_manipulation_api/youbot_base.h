/* *****************************************************************
 *
 * luh_youbot_manipulation_api
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
 * @author Simon Aden (simon.aden@mailbox.org)
 * @date   November 2014
 *
 * @brief  Contains the YoubotBase class.
 */

#ifndef LUH_YOUBOT_MANIPULATION_API_BASE_H
#define LUH_YOUBOT_MANIPULATION_API_BASE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <luh_youbot_msgs/MoveToCartesianPoseAction.h>
#include <luh_youbot_msgs/MoveToCylindricPoseAction.h>
#include <luh_youbot_msgs/MoveToJointPoseAction.h>
#include <luh_youbot_msgs/MoveToNamedPoseAction.h>
#include <luh_youbot_msgs/SetGripperAction.h>
#include <luh_youbot_msgs/GripObjectAction.h>
#include <luh_youbot_msgs/MoveBaseAction.h>
#include <luh_youbot_kinematics/arm_kinematics.h>

namespace manipulation_api
{
/**
 * @brief Struct for a 2-dimensional pose.
 */
struct Pose2D
{
    double x, y, theta;
};

typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief This is a wrapper class for all action clients that control the youBot base.
 * @details Example usage:
 * @code
 * // initialise ROS
 * ros::init(argc, argv, "example node");
 * ros::NodeHandle node_handle;
 *
 * // initialise base
 * manipulation_api::YoubotBase base;
 * base.init(node_handle);
 *
 * // move 50 cm forward
 * base.move(0.5, 0, 0);
 * base.waitForCurrentAction();
 *
 * // turn 180 degrees
 * base.move(0, 0, 3.14);
 * base.waitForCurrentAction();
 *
 * // move 50 cm forward again
 * base.move(0.5, 0, 0);
 * base.waitForCurrentAction();
 *
 * // get traveled distance
 * manipulation_api::Pose2D dist = base.getTraveledDistance();
 *
 * // dist.x and dist.y should be about zero now and dist.theta about 3.14.
 *
 * @endcode
 */
class YoubotBase
{
public:
    /**
     * @brief YoubotBase constructor. Does not initialise the action clients.
     * @details The object must be initialised with the init() method before it can be used.
     * @see YoubotBase#init
     */
    YoubotBase();
    ~YoubotBase();

    /**
     * @brief Initialises all action clients.
     * @param node Node Handle.
     */
    void init(ros::NodeHandle &node);

    /**
     * @brief Moves the base to the specified pose relative to the current position.
     * @param x Distance in x direction (forward) in meters.
     * @param y Distance in y direction (left) in meters.
     * @param theta Angle to turn counter-clockwise in radians.
     */
    void move(double x, double y, double theta);

    /**
     * @brief Moves the base to the specified pose relative to the current position.
     * @param pose Relative pose.
     */
    void move(Pose2D pose);

    /**
     * @brief Waits until the current action has finished.
     * @param timeout Optional timeout in seconds after which the action will be aborted.
     * @return True if the action finished before timeout.
     */
    bool waitForCurrentAction(double timeout=0);

    /**
     * @brief Checks if the base is busy i.e. if an action is currently being executed.
     * @return True if the base is busy.
     */
    bool isBusy(){return is_busy_;}

    /**
     * @brief Aborts the currently executed action.
     */
    void abortCurrentAction();

    /**
     * @brief Checks if the last action has finished successfully.
     * @return True if last action has succeeded.
     */
    bool actionSucceeded();

    /**
     * @brief Returns the accumulated traveled distance since start or last reset.
     * @return Accumulated traveled distance.
     */
    Pose2D getTraveledDistance();

    /**
     * @brief Resets the accumulated traveled distance to zero.
     */
    void resetTraveledDistance();

    /**
     * @brief Returns the youbot's current pose in the odometry frame
     * @return Current Pose in odometry frame.
     */
    Pose2D getCurrentPose();

protected:
    enum ActiveClient
    {
        MOVE_BASE,
        APPROACH // TODO

    } active_client_;

    ros::NodeHandle *node_;

    MoveBaseClient *move_base_client_;

    ros::ServiceClient get_pose_client_;

    Pose2D traveled_distance_;

    bool is_busy_;

    void onMoveBaseActionDone(const actionlib::SimpleClientGoalState& state,
                              const luh_youbot_msgs::MoveBaseResultConstPtr& result);

};

}

#endif // LUH_YOUBOT_MANIPULATION_API_BASE_H
