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

#include "luh_youbot_gazebo/plugins/arm_controller.h"

namespace gazebo 
{

  YoubotArmController::YoubotArmController() {}

  YoubotArmController::~YoubotArmController() {}

  // Load the controller
  void YoubotArmController::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {
    parent_ = parent;

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

    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("YoubotArmController (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

    //TODO: make topic name depend on arm name

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<luh_youbot_msgs::JointVector>("joint_position_command", 1,
          boost::bind(&YoubotArmController::posCmdCallback, this, _1),
          ros::VoidPtr(), &queue_);
    pos_sub_ = rosnode_->subscribe(so);
    so =
          ros::SubscribeOptions::create<luh_youbot_msgs::JointVector>("joint_velocity_command", 1,
              boost::bind(&YoubotArmController::velCmdCallback, this, _1),
              ros::VoidPtr(), &queue_);
    vel_sub_ = rosnode_->subscribe(so);
    so =
          ros::SubscribeOptions::create<luh_youbot_msgs::JointVector>("joint_torque_command", 1,
              boost::bind(&YoubotArmController::trqCmdCallback, this, _1),
              ros::VoidPtr(), &queue_);
    trq_sub_ = rosnode_->subscribe(so);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&YoubotArmController::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&YoubotArmController::UpdateChild, this));

    // initialise command (start with zero velocity)
    cmd_.q1 = 0;
    cmd_.q2 = 0;
    cmd_.q3 = 0;
    cmd_.q4 = 0;
    cmd_.q5 = 0;
    cmd_mode_ = CMD_VELOCITY;

    // get arm joints
    arm_joints_.push_back(parent_->GetJoint("arm_joint_1"));
    arm_joints_.push_back(parent_->GetJoint("arm_joint_2"));
    arm_joints_.push_back(parent_->GetJoint("arm_joint_3"));
    arm_joints_.push_back(parent_->GetJoint("arm_joint_4"));
    arm_joints_.push_back(parent_->GetJoint("arm_joint_5"));

    for(uint i=0; i<arm_joints_.size(); i++)
        //arm_joints_[i]->SetMaxForce(0, 100);
        arm_joints_[i]->SetEffortLimit(0, 100);
  }

  // Update the controller
  void YoubotArmController::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(lock);

    if(cmd_mode_ == CMD_POSITION)
    {
//        arm_joints_[0]->SetAngle(0, math::Angle(cmd_.q1));
//        arm_joints_[1]->SetAngle(0, math::Angle(cmd_.q2));
//        arm_joints_[2]->SetAngle(0, math::Angle(cmd_.q3));
//        arm_joints_[3]->SetAngle(0, math::Angle(cmd_.q4));
//        arm_joints_[4]->SetAngle(0, math::Angle(cmd_.q5));
        arm_joints_[0]->SetPosition(0, cmd_.q1);
        arm_joints_[1]->SetPosition(0, cmd_.q2);
        arm_joints_[2]->SetPosition(0, cmd_.q3);
        arm_joints_[3]->SetPosition(0, cmd_.q4);
        arm_joints_[4]->SetPosition(0, cmd_.q5);
    }
    else if(cmd_mode_ == CMD_VELOCITY)
    {
        arm_joints_[0]->SetVelocity(0, cmd_.q1);
        arm_joints_[1]->SetVelocity(0, cmd_.q2);
        arm_joints_[2]->SetVelocity(0, cmd_.q3);
        arm_joints_[3]->SetVelocity(0, cmd_.q4);
        arm_joints_[4]->SetVelocity(0, cmd_.q5);
    }
    else if(cmd_mode_ == CMD_TORQUE)
    {
        arm_joints_[0]->SetForce(0, cmd_.q1);
        arm_joints_[1]->SetForce(0, cmd_.q2);
        arm_joints_[2]->SetForce(0, cmd_.q3);
        arm_joints_[3]->SetForce(0, cmd_.q4);
        arm_joints_[4]->SetForce(0, cmd_.q5);
    }
  }

  // Finalize the controller
  void YoubotArmController::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void YoubotArmController::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }  

  void YoubotArmController::posCmdCallback(const luh_youbot_msgs::JointVector::ConstPtr &pos)
  {
      cmd_mode_ = CMD_POSITION;
      cmd_ = *pos;
  }

  void YoubotArmController::velCmdCallback(const luh_youbot_msgs::JointVector::ConstPtr &vel)
  {
      cmd_mode_ = CMD_VELOCITY;
      cmd_ = *vel;
  }

  void YoubotArmController::trqCmdCallback(const luh_youbot_msgs::JointVector::ConstPtr &trq)
  {
      cmd_mode_ = CMD_TORQUE;
      cmd_ = *trq;
  }

  GZ_REGISTER_MODEL_PLUGIN(YoubotArmController)
}
