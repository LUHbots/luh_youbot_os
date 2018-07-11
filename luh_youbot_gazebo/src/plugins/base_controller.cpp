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

#include "luh_youbot_gazebo/plugins/base_controller.h"

namespace gazebo 
{

  YoubotBaseController::YoubotBaseController() {}

  YoubotBaseController::~YoubotBaseController() {}

  // Load the controller
  void YoubotBaseController::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {
    parent_ = parent;
    base_link_ = parent_->GetLink("base_footprint");

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("YoubotBaseController missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    } 
    else 
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    } 

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    } 
 
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
    last_odom_pose_ = parent_->WorldPose();
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("YoubotBaseController (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&YoubotBaseController::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&YoubotBaseController::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&YoubotBaseController::UpdateChild, this));

    // get wheel joints
    wheel_joints_.push_back(parent_->GetJoint("wheel_joint_fl"));
    wheel_joints_.push_back(parent_->GetJoint("wheel_joint_fr"));
    wheel_joints_.push_back(parent_->GetJoint("wheel_joint_bl"));
    wheel_joints_.push_back(parent_->GetJoint("wheel_joint_br"));

    // disable force limits to allow velocity control
    for(uint i=0; i<wheel_joints_.size(); i++)
        //wheel_joints_[i]->SetMaxForce(0, HUGE_VAL);
        wheel_joints_[i]->SetEffortLimit(0, HUGE_VAL);


    // base kinematics
    double r = 0.0475;
    double b = 0.3;
    double l = 0.47;
    base_kin_inv_.assign(12, 1.0 / r);
    base_kin_inv_[0]  *=  1.0;
    base_kin_inv_[1]  *= -1.0;
    base_kin_inv_[2]  *= -0.5 * (b + l);
    base_kin_inv_[3]  *=  1.0;
    base_kin_inv_[4]  *=  1.0;
    base_kin_inv_[5]  *=  0.5 * (b + l);
    base_kin_inv_[6]  *=  1.0;
    base_kin_inv_[7]  *=  1.0;
    base_kin_inv_[8]  *= -0.5 * (b + l);
    base_kin_inv_[9]  *=  1.0;
    base_kin_inv_[10] *= -1.0;
    base_kin_inv_[11] *=  0.5 * (b + l);

  }

  // Update the controller
  void YoubotBaseController::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(lock);

    // set cartesian velocity
    ignition::math::Pose3d pose = base_link_->WorldPose();
    float roll = pose.Rot().Roll();
    float pitch = pose.Rot().Pitch();
    float yaw = pose.Rot().Yaw();
    
    //test
    //ROS_INFO("%.2f %.2f %.2f %.2f %.2f %.2f",pose.pos.x,pose.pos.y,pose.pos.z,roll,pitch,yaw);
    //if (pose.pos.z >= 0.09){
    //    base_link_->SetWorldPose(math::Pose(pose.pos.x,pose.pos.y,0.08,roll,pitch,yaw));
    //}
    
    double xt = x_ * cosf(yaw) - y_ * sinf(yaw);
    double yt = y_ * cosf(yaw) + x_ * sinf(yaw);
    double vel_factor = 1.0;

    base_link_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, rot_));
    base_link_->SetLinearVel(ignition::math::Vector3d(xt, yt, -0.0));

    // set wheel velocity
    /*
    wheel_joints_[0]->SetVelocity(0, base_kin_inv_[0] * x_ + base_kin_inv_[1]  * y_ + base_kin_inv_[2]  * rot_);
    wheel_joints_[1]->SetVelocity(0, base_kin_inv_[3] * x_ + base_kin_inv_[4]  * y_ + base_kin_inv_[5]  * rot_);
    wheel_joints_[2]->SetVelocity(0, base_kin_inv_[6] * x_ + base_kin_inv_[7]  * y_ + base_kin_inv_[8]  * rot_);
    wheel_joints_[3]->SetVelocity(0, base_kin_inv_[9] * x_ + base_kin_inv_[10] * y_ + base_kin_inv_[11] * rot_);*/

    // publish odometry
    if (odometry_rate_ > 0.0) {
      common::Time current_time = parent_->GetWorld()->SimTime();
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void YoubotBaseController::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void YoubotBaseController::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
  }

  void YoubotBaseController::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void YoubotBaseController::publishOdometry(double step_time)
  {

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
    ignition::math::Pose3d pose = this->parent_->WorldPose();

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame,
            base_footprint_frame));

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
    linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
    if (rot_ > M_PI / step_time) 
    { 
      // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = rot_;
    } 
    else 
    {
      float last_yaw = last_odom_pose_.Rot().Yaw();
      float current_yaw = pose.Rot().Yaw();
      while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
      while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_pub_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(YoubotBaseController)
}
