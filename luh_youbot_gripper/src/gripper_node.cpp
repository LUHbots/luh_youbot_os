#include "luh_youbot_gripper/gripper_node.h"
#include "ros/package.h"

namespace luh_youbot_gripper
{

GripperNode::GripperNode(ros::NodeHandle &node):
    node_(&node)
{    
    double frequency;

    // === PARAMETERS ===
    node.param("luh_gripper/min_gripper_width", min_gripper_width_, 0.0);
    node.param("luh_gripper/max_gripper_width", max_gripper_width_, 0.06);
    node.param("luh_gripper/grip_force", grip_force_, 80.0);
    node.param("luh_gripper/position_tolerance", position_tolerance_, 0.003);
    node.param("luh_gripper/force_tolerance", force_tolerance_, 4.0);
    node.param("luh_gripper/open_force", open_force_, 1000.0);
    node.param("luh_gripper/frequency", frequency, 50.0);
    node.param("luh_gripper/min_angle", min_angle_, 0.0);
    node.param("luh_gripper/max_angle", max_angle_, 80.0);
    node.param("luh_gripper/static_grip_force", static_grip_force_, 20.0);
    node.param("luh_gripper/use_voltage_log", use_voltage_log_, false);

    // === ACTION SERVER ===
    grip_object_server_ = new GripObjectServer(*node_, "arm_1/grip_object", false);
    grip_object_server_->registerGoalCallback(boost::bind(&GripperNode::gripObjectCallback, this));
    grip_object_server_->start();
    set_gripper_server_ = new SetGripperServer(*node_, "arm_1/set_gripper", false);
    set_gripper_server_->registerGoalCallback(boost::bind(&GripperNode::setGripperCallback, this));
    set_gripper_server_->start();

    // === SUBSCRIBER ===
    gripper_pos_cmd_subscriber_ = node_->subscribe(
                "arm_1/gripper_command", 1, &GripperNode::gripperCommandCallback, this);

    // === PUBLISHER ===
    joint_state_publisher_ = node.advertise<sensor_msgs::JointState>("gripper/joint_states",1);

    // === USB-Vcc Publischer ===

    usb_voltage_publisher_= node.advertise<std_msgs::Float32>("usb_voltage",1);

    // === TIMER ===
    timer_ = node.createTimer(ros::Duration(1.0/frequency),
                              &GripperNode::timerCallback, this, false, false);

    // === SERVICE SERVER ===
    grip_check_server_ = node.advertiseService("gripper/check", &GripperNode::gripCheckCallback, this);

    // === INIT GRIPPER ===
    gripper_.initialise();

    // === INIT JOINT STATE MESSAGE ===
    joint_state_msg_.effort.assign(2, 0);
    joint_state_msg_.position.assign(2, 0);
    joint_state_msg_.velocity.assign(2, 0);
    joint_state_msg_.name.push_back("gripper_finger_joint_l");
    joint_state_msg_.name.push_back("gripper_finger_joint_r");
    joint_state_msg_.header.frame_id = "gripper_tip";

    // === INIT USB VOLTAGE LOGGING
    if(use_voltage_log_)
    {
        std::string filename = ros::package::getPath(ROS_PACKAGE_NAME);
        filename.append("/log/USB_VCC_LOG.csv");
        LogFile.open (filename.c_str(), std::ofstream::app);
    }

    ros::Duration(1.0).sleep();
    ROS_INFO("Calibrating gripper...");
    gripper_.setMaxEffort(10000, 10000);
    gripper_.setPosition(0, 0);
    ros::Duration(2.0).sleep();
    gripper_.setPosition(-max_angle_, max_angle_);
    ros::Duration(2.0).sleep();

    timer_.start();
}

//########## TIMER CALLBACK ############################################################################################
void GripperNode::timerCallback(const ros::TimerEvent &evt)
{
    // get joint state
    joint_state_msg_.header.stamp = ros::Time::now();

    GripperState state = gripper_.getState();

    joint_state_msg_.position[0] = (state.position.first + max_angle_) *(max_gripper_width_ - min_gripper_width_) / (max_angle_ - min_angle_) + min_gripper_width_;
    joint_state_msg_.position[1] = (state.position.second - min_angle_) *(max_gripper_width_ - min_gripper_width_) / (max_angle_ - min_angle_) + min_gripper_width_;
    joint_state_msg_.velocity[0] = state.velocity.first;
    joint_state_msg_.velocity[1] = state.velocity.second;
    joint_state_msg_.effort[0] = state.effort.first;
    joint_state_msg_.effort[1] = state.effort.second;

    // publish joint state
    joint_state_publisher_.publish(joint_state_msg_);

    //publish usb-voltage

    usb_voltage_msg_.data=state.usbVcc/100.0;

    if(use_voltage_log_)
    {
//    timeval TimeNow;
//    gettimeofday(&TimeNow, NULL);
//    unsigned long long currentTime =
//        (unsigned long long)(TimeNow.tv_sec) * 1000 +
//        (unsigned long long)(TimeNow.tv_usec) / 1000;


    ros::Time time_now = ros::Time::now();
    LogFile << time_now.toSec();
    LogFile <<";"<< state.usbVcc/100.0 << "\n";
    }
    usb_voltage_publisher_.publish(usb_voltage_msg_);



    current_position_ = joint_state_msg_.position[0] + joint_state_msg_.position[1];

    if(state.goal_reached)
    {
        if(set_gripper_server_->isActive())
        {
            set_gripper_server_->setSucceeded();
        }
        if(grip_object_server_->isActive())
        {
            grip_object_server_->setSucceeded();
        }
    }
}

//########## GRIPPER COMMAND CALLBACK ##################################################################################
void GripperNode::gripperCommandCallback(const std_msgs::Float32::ConstPtr &msg)
{
    setGripperPosition(msg->data);
}

//########## SET GRIPPER POSITION ######################################################################################
void GripperNode::setGripperPosition(double position, bool is_relative)
{
    if(!is_relative && (position < min_gripper_width_ || position > max_gripper_width_))
    {
        ROS_ERROR("Position is out of range.");
        return;
    }

    // set max effort
    if(is_relative)
    {
        if(position > 0)
            gripper_.setMaxEffort(open_force_, open_force_);
        else
            gripper_.setMaxEffort(grip_force_, grip_force_);
    }
    else
    {
        if(position > current_position_)
            gripper_.setMaxEffort(open_force_, open_force_);
        else
            gripper_.setMaxEffort(grip_force_, grip_force_);
    }

    // set position
    double angle = positionToAngle(position);
    if(is_relative)
        gripper_.setRelativePosition(-angle, angle);
    else
        gripper_.setPosition(-angle, angle);
}

//########## CALLBACK: GRIP OBJECT #####################################################################################
void GripperNode::gripObjectCallback()
{
    ROS_INFO("==== Module Gripper ====");

    std::string name = grip_object_server_->acceptNewGoal()->object_name;

    if(name.compare("OPEN") == 0)
    {
        gripper_.setMaxEffort(open_force_, open_force_);
        short angle = positionToAngle(max_gripper_width_);
        gripper_.setPosition(-angle, angle);
    }
    else
    {
        gripper_.setMaxEffort(grip_force_, grip_force_);
        short angle = positionToAngle(min_gripper_width_);
        gripper_.setPosition(-angle, angle);
    }

    ROS_INFO("Gripper command received.");
}

//########## CALLBACK: SET GRIPPER (ACTION) ############################################################################
void GripperNode::setGripperCallback()
{
    ROS_INFO("==== Module Gripper ====");

    luh_youbot_msgs::SetGripperGoal::ConstPtr goal = set_gripper_server_->acceptNewGoal();

    double new_goal_width = goal->gripper_width;


    if(!goal->is_relative && (new_goal_width > max_gripper_width_ || new_goal_width < min_gripper_width_))
    {
        //TODO: check relative position too?

        ROS_ERROR("Requested gripper width of %f is out of range.", new_goal_width);

        set_gripper_server_->setAborted();

        return;
    }

    setGripperPosition(new_goal_width, goal->is_relative);

    ROS_INFO("Gripper command received.");
}

//########## POSITION TO ANGLE #########################################################################################
short GripperNode::positionToAngle(double position)
{
    return (position - min_gripper_width_) * (max_angle_ - min_angle_) / (max_gripper_width_ - min_gripper_width_) + min_angle_;
}

//########## GRIP CHECK CALLBACK #######################################################################################
bool GripperNode::gripCheckCallback(GripCheck::Request &req, GripCheck::Response &res)
{
    double current_force = fabs(joint_state_msg_.effort[0]) + fabs(joint_state_msg_.effort[1]);

    res.has_object = current_force > static_grip_force_ * 2;

    return true;
}

} // namespace luh_youbot_gripper
