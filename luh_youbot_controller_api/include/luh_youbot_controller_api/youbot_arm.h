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
 * @brief  Contains the YoubotArm class.
 */

#ifndef LUH_YOUBOT_CONTROLLER_API_ARM_H
#define LUH_YOUBOT_CONTROLLER_API_ARM_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <luh_youbot_msgs/MoveToCartesianPoseAction.h>
#include <luh_youbot_msgs/MoveToCylindricPoseAction.h>
#include <luh_youbot_msgs/MoveToJointPoseAction.h>
#include <luh_youbot_msgs/MoveToNamedPoseAction.h>
#include <luh_youbot_msgs/SetGripperAction.h>
#include <luh_youbot_msgs/GripObjectAction.h>
#include <luh_youbot_msgs/MoveJointPathAction.h>
#include <luh_youbot_msgs/MoveCartesianPathAction.h>
#include <luh_youbot_msgs/MoveCylindricPathAction.h>
#include <luh_youbot_msgs/MoveNamedPathAction.h>
#include <luh_youbot_kinematics/arm_kinematics.h>

/// Controller API
namespace youbot_api
{

typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveToCartesianPoseAction> CartesianPoseClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveToCylindricPoseAction> CylindricPoseClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveToJointPoseAction> JointPoseClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveToNamedPoseAction> NamedPoseClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveJointPathAction> JointPathClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveCylindricPathAction> CylindricPathClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveCartesianPathAction> CartesianPathClient;
typedef actionlib::SimpleActionClient<luh_youbot_msgs::MoveNamedPathAction> NamedPathClient;

/// Motion Mode
namespace MotionMode
{
/**
 * @brief Movement mode enum
 */
enum MotionMode{
    /// Direct motion without interpolation
    DIRECT,
    /// Interpolated motion
    INTERPOLATE,
    /// Path planning
    PLAN
};
}

/// Interpolation Mode
namespace InterpolationMode
{
/**
 * @brief Interpolation mode for interpolated movements.
 */
enum InterpolationMode
{
    /// Interpolation in jointspace coordinates
    JOINTSPACE,
    /// Interpolation in cylindrical coordinates
    CYLINDRIC,
    /// Interpolation in cartesian coordinates
    CARTESIAN
};
}

/// Action Client
namespace ActionClient
{
/**
 * @brief Enum for the action clients. Needed to enable/disable single clients.
 */
enum ActionClient
{
    /// Cartesian coordinates, DIRECT
    CART_DIRECT,
    /// Cartesian coordinates, INTER
    CART_INTER,
    /// Cartesian coordinates, PLAN
    CART_PLAN,
    /// Cartesian coordinates, Path
    CART_PATH,
    /// Cylindrical coordinates, DIRECT
    CYL_DIRECT,
    /// Cylindrical coordinates, INTER
    CYL_INTER,
    /// Cylindrical coordinates, PLAN
    CYL_PLAN,
    /// Cylindrical coordinates, Path
    CYL_PATH,
    /// Jointspace coordinates, DIRECT
    JOINT_DIRECT,
    /// Jointspace coordinates, INTER
    JOINT_INTER,
    /// Jointspace coordinates, PLAN
    JOINT_PLAN,
    /// Jointspace coordinates, Path
    JOINT_PATH,
    /// Predefined pose, DIRECT
    NAME_DIRECT,
    /// Predefined pose, INTER
    NAME_INTER,
    /// Predefined pose, PLAN
    NAME_PLAN,
    /// Predefined pose, Path
    NAME_PATH
};
}

/// Path in jointspace coordinates
typedef std::vector<luh_youbot_kinematics::JointPosition> JointPath;
/// Path in cylindrical coordinates
typedef std::vector<luh_youbot_kinematics::CylindricPosition> CylindricPath;
/// Path in cartesian coordinates
typedef std::vector<luh_youbot_kinematics::CartesianPosition> CartesianPath;
/// Path with predefined poses
typedef std::vector<std::string> NamedPath;

/**
 * @brief This is a wrapper class for all action clients that control the youBot arm.
 * @details Example usage:
 * @code
 * // initialise ROS
 * ros::init(argc, argv, "example node");
 * ros::NodeHandle node_handle;
 *
 * // initialise gripper
 * youbot_api::YoubotArm arm;
 * arm.init(node_handle);
 *
 * // move to ARM_UP pose with jointspace interpolation
 * arm.moveToPose("ARM_UP", MotionMode::INTERPOLATE, InterpolationMode::JOINTSPACE);
 * arm.waitForCurrentAction();
 *
 * // move to cartesian position with default parameters
 * luh_youbot_kinematics::CartesianPosition pos;
 * pos.setX(0.2);
 * pos.setY(0.0); // actually not neccessary because values are initialised with zero (same for q5)
 * pos.setZ(0.01);
 * pos.setTheta(M_PI);
 * pos.setQ5(0.0);
 *
 * arm.moveToPose(pos);
 *
 * while(arm.isBusy())
 * {
 * // do something else...
 * }
 *
 * // move 5cm down with interpolated movement
 * luh_youbot_kinematics::CylindricPosition relative_pos;
 * relative_pos.setZ(-0.05);
 *
 * arm.moveToPose(relative_pos, youbot_api::MotionMode::INTERPOLATE, true);
 *
 * bool has_finished = arm.waitForCurrentAction();
 *
 * // check for errors
 * if(!has_finished)
 *    ROS_ERROR("Arm movement timed out.");
 *
 * if(!arm.actionSuceeded())
 *    ROS_ERROR("Arm movement failed.");
 *
 * @endcode
 */
class YoubotArm
{
public:
    /**
     * @brief YoubotArm constructor. Does not initialise the action clients.
     * @details The object must be initialised with the init() method before it can be used.
     * @see YoubotArm#init.
     */
    YoubotArm();    

    ~YoubotArm();

    /**
     * @brief Enable the specified action client.
     * @details This method only has effect if it is called before init(). By default all action clients are enabled.
     *          Because there are many action clients it might be useful to only enable the ones that are actually
     *          needed to avoid unneccesary connections. If you are not sure don't worry about this and let all clients
     *          enabled.
     * @param client Specifies the action client. See enum ActionClient.
     * @see YoubotArm#disableClient, YoubotArm#enableAllClients, YoubotArm#disableAllClients
     */
    void enableClient(int client);

    /**
     * @brief Disable the specified action client.
     * @details This method only has effect if it is called before init(). By default all action clients are enabled.
     *          Because there are many action clients it might be useful to only enable the ones that are actually
     *          needed to avoid unneccesary connections. If you are not sure don't worry about this and let all clients
     *          enabled.
     * @param client Specifies the action client. See enum ActionClient.
     * @see YoubotArm#enableClient, YoubotArm#enableAllClients, YoubotArm#disableAllClients
     */
    void disableClient(int client);

    /**
     * @brief Enables all action clients.
     * @details This method only has effect if it is called before init(). By default all action clients are enabled.
     *          Because there are many action clients it might be useful to only enable the ones that are actually
     *          needed to avoid unneccesary connections. If you are not sure don't worry about this and let all clients
     *          enabled.
     * @param client Specifies the action client. See enum ActionClient.
     * @see YoubotArm#enableClient, YoubotArm#disableClient, YoubotArm#disableAllClients
     */
    void enableAllClients();

    /**
     * @brief Disables all action clients.
     * @details This method only has effect if it is called before init(). By default all action clients are enabled.
     *          Because there are many action clients it might be useful to only enable the ones that are actually
     *          needed to avoid unneccesary connections. If you are not sure don't worry about this and let all clients
     *          enabled.
     * @param client Specifies the action client. See enum ActionClient.
     * @see YoubotArm#enableClient, YoubotArm#disableClient, YoubotArm#enableAllClients
     */
    void disableAllClients();

    /**
     * @brief Initialises all enabled action clients.
     * @param node Node handle.
     */
    void init(ros::NodeHandle &node);

    /**
     * @brief Moves the arm to the specified joint positions.
     * @param pose Goal position in jointspace.
     * @param mode Movement mode. See enum MotionMode.
     * @param pose_is_relative Specifies if the pose is relative to current pose or absolute.
     */
    void moveToPose(luh_youbot_kinematics::JointPosition pose, int mode=MotionMode::PLAN,
                    bool pose_is_relative=false);

    /**
     * @brief Moves the endeffector to the specified position in cylindrical coordinates.
     * @param pose Goal position in cylindrical coordinates.
     * @param mode Movement mode. See enum MotionMode.
     * @param pose_is_relative Specifies if the pose is relative to current pose or absolute.
     */
    void moveToPose(luh_youbot_kinematics::CylindricPosition pose, int mode=MotionMode::PLAN,
                    bool pose_is_relative=false);

    /**
     * @brief Moves the endeffector to the specified position in cartesian coordinates.
     * @param pose Goal position in cartesian coordinates.
     * @param mode Movement mode. See enum MotionMode.
     * @param frame_id Frame ID.
     * @param pose_is_relative Specifies if the pose is relative to current pose or absolute.
     */
    void moveToPose(luh_youbot_kinematics::CartesianPosition pose, int mode=MotionMode::PLAN,
                    std::string frame_id="arm_link_0",
                    bool pose_is_relative=false);

    /**
     * @brief Moves the arm to a predefined pose.
     * @param pose_name Name of the pose.
     * @param movement_mode Movement mode. See enum MotionMode.
     * @param interpolation_mode Interpolation mode. See enum InterpolationMode.
     */
    void moveToPose(std::string pose_name, int movement_mode=MotionMode::PLAN,
                    int interpolation_mode=InterpolationMode::JOINTSPACE);

    /**
     * @brief Lets the arm follow specified path.
     * @details This action is for continous path movements with a few  key points. Poses between the key points
     *          are interpolated in jointspace coordinates. If you need an exact trajectory with many path points
     *          use the FollowJointPath action.
     * @param path Path in jointspace coordinates.
     */
    void moveAlongPath(JointPath path);

    /**
     * @brief Lets the arm follow specified path.
     * @details This action is for continous path movements with a few  key points. Poses between the key points
     *          are interpolated in jointspace coordinates. If you need an exact trajectory with many path points
     *          use the FollowJointPath action.
     * @param path Path in cylindrical coordinates.
     */
    void moveAlongPath(CylindricPath path);

    /**
     * @brief Lets the arm follow specified path.
     * @details This action is for continous path movements with a few  key points. Poses between the key points
     *          are interpolated in jointspace coordinates. If you need an exact trajectory with many path points
     *          use the FollowJointPath action.
     * @param path Path in cartesian coordinates.
     */
    void moveAlongPath(CartesianPath path);

    /**
     * @brief Lets the arm follow specified path.
     * @details This action is for continous path movements with a few  key points. Poses between the key points
     *          are interpolated in jointspace coordinates. If you need an exact trajectory with many path points
     *          use the FollowJointPath action.
     * @param path Path with predefined poses.
     */
    void moveAlongPath(NamedPath path);

    /**
     * @brief Waits until the current action is finished.
     * @param timeout Optional timeout in seconds after which the action will be aborted.
     * @return True if the action finished before timeout.
     */
    bool waitForCurrentAction(double timeout=0);

    /**
     * @brief Checks if the arm is busy i.e. if an action is currently executed.
     * @return True if the arm is busy.
     */
    bool isBusy(){return arm_is_busy_;}

    /**
     * @brief Aborts the currently executed action.
     */
    void abortCurrentAction();

    /**
     * @brief Checks if the last action has succeeded.
     * @return True if the last action has finished successfully.
     */
    bool actionSucceeded();

    /**
     * @brief Sets the maximum velocity for interpolated movements in jointspace.
     * @param velocity Maximum velocity.
     * @return Returns false if the values are invalid.
     */
    bool setMaxVelocity(luh_youbot_kinematics::JointVelocity velocity);

    /**
     * @brief Sets the maximum velocity for interpolated movements in cartesian coordinates.
     * @param velocity Maximum velocity.
     * @return Returns false if the values are invalid.
     */
    bool setMaxVelocity(luh_youbot_kinematics::CartesianVelocity velocity);

    /**
     * @brief Sets the maximum velocity for interpolated movements in cylindrical coordiantes.
     * @param velocity Maximum velocity.
     * @return Returns false if the values are invalid.
     */
    bool setMaxVelocity(luh_youbot_kinematics::CylindricVelocity velocity);

protected:
    ActionClient::ActionClient active_client_;

    ros::NodeHandle *node_;

    CartesianPoseClient *cart_client_direct_;
    CartesianPoseClient *cart_client_inter_;
    CartesianPoseClient *cart_client_plan_;

    CylindricPoseClient *cyl_client_direct_;
    CylindricPoseClient *cyl_client_inter_;
    CylindricPoseClient *cyl_client_plan_;

    JointPoseClient *jnt_client_direct_;
    JointPoseClient *jnt_client_inter_;
    JointPoseClient *jnt_client_plan_;

    NamedPoseClient *name_client_direct_;
    NamedPoseClient *name_client_inter_;
    NamedPoseClient *name_client_plan_;

    CartesianPathClient *cart_traj_client_;
    CylindricPathClient *cyl_traj_client_;
    JointPathClient *jnt_traj_client_;
    NamedPathClient *name_traj_client_;

    ros::ServiceClient set_cart_vel_client_;
    ros::ServiceClient set_cyl_vel_client_;
    ros::ServiceClient set_jnt_vel_client_;

    bool arm_is_busy_;

    std::map<int, bool> is_enabled_;

    void onJointActionDone(const actionlib::SimpleClientGoalState& state,
                           const luh_youbot_msgs::MoveToJointPoseResultConstPtr& result);
    void onCartActionDone(const actionlib::SimpleClientGoalState& state,
                          const luh_youbot_msgs::MoveToCartesianPoseResultConstPtr& result);
    void onCylActionDone(const actionlib::SimpleClientGoalState& state,
                         const luh_youbot_msgs::MoveToCylindricPoseResultConstPtr& result);
    void onNamedActionDone(const actionlib::SimpleClientGoalState& state,
                           const luh_youbot_msgs::MoveToNamedPoseResultConstPtr& result);

    void onJointTrajActionDone(const actionlib::SimpleClientGoalState& state,
                           const luh_youbot_msgs::MoveJointPathResultConstPtr& result);
    void onCartTrajActionDone(const actionlib::SimpleClientGoalState& state,
                          const luh_youbot_msgs::MoveCartesianPathResultConstPtr& result);
    void onCylTrajActionDone(const actionlib::SimpleClientGoalState& state,
                         const luh_youbot_msgs::MoveCylindricPathResultConstPtr& result);
    void onNamedTrajActionDone(const actionlib::SimpleClientGoalState& state,
                           const luh_youbot_msgs::MoveNamedPathResultConstPtr& result);

};

}

#endif // LUH_YOUBOT_CONTROLLER_API_ARM_H
