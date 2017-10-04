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

#ifndef YOUBOT_GAZEBO_PLUGINS_ARM_CONTROLLER_HH
#define YOUBOT_GAZEBO_PLUGINS_ARM_CONTROLLER_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <luh_youbot_msgs/JointVector.h>

namespace gazebo {

  class YoubotArmController : public ModelPlugin {

    public: 
      YoubotArmController();
      ~YoubotArmController();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:      
      enum CommandMode
      {
          CMD_POSITION,
          CMD_VELOCITY,
          CMD_TORQUE
      }cmd_mode_;

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Subscriber pos_sub_;
      ros::Subscriber vel_sub_;
      ros::Subscriber trq_sub_;

      boost::mutex lock;

      std::string robot_namespace_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void posCmdCallback(const luh_youbot_msgs::JointVector::ConstPtr &pos);
      void velCmdCallback(const luh_youbot_msgs::JointVector::ConstPtr &vel);
      void trqCmdCallback(const luh_youbot_msgs::JointVector::ConstPtr &trq);

      luh_youbot_msgs::JointVector cmd_;
      std::vector<physics::JointPtr> arm_joints_;
      bool alive_;
      bool command_recived_;
  };

}

#endif /* end of include guard: YOUBOT_GAZEBO_PLUGINS_ARM_CONTROLLER_HH */

