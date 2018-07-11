/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */

#include "luh_youbot_gazebo/plugins/gripper_controller.h"

namespace gazebo
{

YoubotGripperController::YoubotGripperController() {}

YoubotGripperController::~YoubotGripperController() {}

// Load the controller
void YoubotGripperController::Load(physics::ModelPtr parent,
                                   sdf::ElementPtr sdf)
{
    parent_ = parent;

    alive_ = true;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("namespace"))
    {
      ROS_INFO("YoubotArmController missing <namespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else
    {
      robot_namespace_ =
        sdf->GetElement("namespace")->Get<std::string>();
    }
    if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";

    rosnode_.reset(new ros::NodeHandle(robot_namespace_));
//    rosnode_->setCallbackQueue(&queue_);

    // === ACTION SERVER ===
    grip_object_server_ = new GripObjectServer(*rosnode_, "arm_1/grip_object", false);
    grip_object_server_->registerGoalCallback(boost::bind(&YoubotGripperController::gripObjectCallback, this));
    grip_object_server_->start();

    set_gripper_server_ = new SetGripperServer(*rosnode_, "arm_1/set_gripper", false);
    set_gripper_server_->registerGoalCallback(boost::bind(&YoubotGripperController::setGripperCallback, this));
    set_gripper_server_->start();

    // === SUBSCRIBER ===
    gripper_pos_cmd_subscriber_ = rosnode_->subscribe(
                "arm_1/gripper_command", 1, &YoubotGripperController::gripperCommandCallback, this);

    // === SERVICE SERVER ===
    grip_check_server_ = rosnode_->advertiseService("gripper/check", &YoubotGripperController::gripCheckCallback, this);

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&YoubotGripperController::UpdateChild, this));

    // initialise command (start with zero velocity)
    pos_cmd_ = 0;
    force_ = 10;
    left_is_opening_ = false;
    right_is_opening_ = false;
    left_pos_reached_ = false;
    right_pos_reached_ = false;

    // get arm joints
    left_joint_ = parent_->GetJoint("gripper_finger_joint_l");
    right_joint_ = parent_->GetJoint("gripper_finger_joint_r");

//    spinner_.reset(new ros::AsyncSpinner(1, &queue_));
//    spinner_->start();
    command_recived_ = false;
}

// Update the controller
void YoubotGripperController::UpdateChild()
{
    boost::mutex::scoped_lock scoped_lock(lock);
    if(command_recived_)
    {
    if(!left_pos_reached_)
    {
        double left_pos = left_joint_->Position(0);//.Radian();
        if(left_is_opening_ && left_pos < 0.5 * pos_cmd_)
        {
            //ROS_INFO("Setting left opening");
            left_joint_->SetForce(0, force_);
        }
        else if(!left_is_opening_ && left_pos > 0.5 * pos_cmd_)
        {
            //ROS_INFO("Setting left closing Pos: %.3f pos_cmd: %.3f",left_pos,pos_cmd_);
            left_joint_->SetForce(0, -force_);
        }
        else
        {
            if(command_recived_)
            {
            //left_joint_->SetAngle(0, math::Angle(0.5 * pos_cmd_));
            left_joint_->SetPosition(0, 0.5 * pos_cmd_);
            command_recived_ = false;
            }
        }
    }

    if(!right_pos_reached_)
    {
        double right_pos = right_joint_->Position(0);//.Radian();
        if(right_is_opening_ && right_pos < 0.5 * pos_cmd_)
        {
            //ROS_INFO("Setting right opening");
            right_joint_->SetForce(0, force_);
        }
        else if(!right_is_opening_ && right_pos > 0.5 * pos_cmd_)
        {
            //ROS_INFO("Setting right closing Pos: %.3f pos_cmd: %.3f",right_pos,pos_cmd_);
            right_joint_->SetForce(0, -force_);
        }
        else
        {
            //right_joint_->SetAngle(0, math::Angle(0.5 * pos_cmd_));
            right_joint_->SetPosition(0, 0.5);
        }
    }
    }
    // TODO: check if gripper is closed
    if(set_gripper_server_->isActive())
        set_gripper_server_->setSucceeded();
    if(grip_object_server_->isActive())
        grip_object_server_->setSucceeded();

}

// Finalize the controller
void YoubotGripperController::FiniChild() {
    alive_ = false;
//    spinner_->stop();
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    //    callback_queue_thread_.join();
}

//  void YoubotGripperController::QueueThread()
//  {
//    static const double timeout = 0.01;
//    while (alive_ && rosnode_->ok())
//    {
//      queue_.callAvailable(ros::WallDuration(timeout));
//    }
//  }

void YoubotGripperController::gripperCommandCallback(const std_msgs::Float32::ConstPtr &msg)
{
    boost::mutex::scoped_lock scoped_lock(lock);

    setCmd(msg->data);
}

bool YoubotGripperController::gripCheckCallback(luh_youbot_gripper::GripCheck::Request &req,
                                                luh_youbot_gripper::GripCheck::Response &res)
{
    boost::mutex::scoped_lock scoped_lock(lock);

    res.has_object = !(left_pos_reached_ && right_pos_reached_);
    return true;
}

void YoubotGripperController::gripObjectCallback()
{
    boost::mutex::scoped_lock scoped_lock(lock);

    std::string name = grip_object_server_->acceptNewGoal()->object_name;

    if(name.compare("OPEN") == 0)
    {
        setCmd(0.06); //TODO parameter
    }
    else
    {
        setCmd(0);
    }
}

void YoubotGripperController::setGripperCallback()
{
    boost::mutex::scoped_lock scoped_lock(lock);

    luh_youbot_msgs::SetGripperGoal::ConstPtr goal = set_gripper_server_->acceptNewGoal();

    if(goal->is_relative)
    {
        double right_pos = right_joint_->Position(0);//.Radian();
        double left_pos = left_joint_->Position(0);//.Radian();
        setCmd(goal->gripper_width + left_pos + right_pos);
    }
    else
    {
        setCmd(goal->gripper_width);
    }
}

void YoubotGripperController::setCmd(double cmd)
{
    pos_cmd_ = cmd;
    command_recived_ = true;
    double right_pos = right_joint_->Position(0);//.Radian();
    right_is_opening_ = right_pos < 0.5 * pos_cmd_;

    double left_pos = left_joint_->Position(0);//.Radian();
    left_is_opening_ = left_pos < 0.5 * pos_cmd_;

    left_pos_reached_ = false;
    right_pos_reached_ = false;
}

GZ_REGISTER_MODEL_PLUGIN(YoubotGripperController)
}
