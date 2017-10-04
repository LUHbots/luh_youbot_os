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

#ifndef YOUBOT_GAZEBO_PLUGINS_GRIPPER_CONTROLLER_HH
#define YOUBOT_GAZEBO_PLUGINS_GRIPPER_CONTROLLER_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <luh_youbot_msgs/GripObjectAction.h>
#include <luh_youbot_msgs/SetGripperAction.h>
#include <luh_youbot_gripper/GripCheck.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>

namespace gazebo {

  class YoubotGripperController : public ModelPlugin {

    public: 

      typedef actionlib::SimpleActionServer<luh_youbot_msgs::GripObjectAction> GripObjectServer;
      typedef actionlib::SimpleActionServer<luh_youbot_msgs::SetGripperAction> SetGripperServer;

      YoubotGripperController();
      ~YoubotGripperController();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

      GripObjectServer* grip_object_server_;
      SetGripperServer* set_gripper_server_;
      ros::Subscriber gripper_pos_cmd_subscriber_;
      ros::ServiceServer grip_check_server_;

    private:      

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;

      boost::mutex lock;
      double pos_cmd_;
      double force_;
      physics::JointPtr left_joint_;
      physics::JointPtr right_joint_;
      bool left_is_opening_;
      bool right_is_opening_;
      bool left_pos_reached_;
      bool right_pos_reached_;
      bool alive_;
      std::string robot_namespace_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
//      boost::thread callback_queue_thread_;
//      void QueueThread();
      boost::shared_ptr<ros::AsyncSpinner> spinner_;

      void gripperCommandCallback(const std_msgs::Float32::ConstPtr &msg);
      bool gripCheckCallback(luh_youbot_gripper::GripCheck::Request &req, luh_youbot_gripper::GripCheck::Response &res);

      void setCmd(double cmd);
      void gripObjectCallback();
      void setGripperCallback();
      bool command_recived_;
  };

}

#endif /* end of include guard: YOUBOT_GAZEBO_PLUGINS_GRIPPER_CONTROLLER_HH */

