#include "luh_youbot_controller/module_example/simple_module_example.h"


//########## CONSTRUCTOR ###############################################################################################
ModuleExample::ModuleExample(): ControllerModule() // don't forget to call base class constuctor!
{
}

//########## INITIALIZATION ############################################################################################
void ModuleExample::init()
{
    activated_ = true;

    // initialise ROS communication
    cartesian_pose_subscriber_ = node_->subscribe("my_topic", 100, &ModuleExample::cartesianPoseCallback, this);
    joint_pose_publisher_ = node_->advertise<luh_youbot_msgs::JointVector>("joint_pose", 100);
}

//########## UPDATE ####################################################################################################
void ModuleExample::update()
{
    if(!activated_)
        return;

    // transform cartesian pose to jointspace (inverse kinematics)
    luh_youbot_kinematics::JointPosition joint_pose = cartesian_pose_.toJointspace(joint_state_position_);

    // create output message
    controller_msgs::JointVector joint_pose_msg;
    joint_pose.subtractOffset();    
    joint_pose_msg.q1 = joint_pose.q1();
    joint_pose_msg.q2 = joint_pose.q2();
    joint_pose_msg.q3 = joint_pose.q3();
    joint_pose_msg.q4 = joint_pose.q4();
    joint_pose_msg.q5 = joint_pose.q5();

    // publish message
    joint_pose_publisher_.publish(joint_pose_msg);
}

//########## ACTIVATE ##################################################################################################
void ModuleExample::activate()
{
    activated_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleExample::deactivate()
{
    activated_ = false;
}

//########## CALLBACK: CARTESIAN POSE ##################################################################################
void ModuleExample::cartesianPoseCallback(const luh_youbot_msgs::CartesianVector::ConstPtr &cartesian_pose_msg)
{
    // read input message
    cartesian_pose_.setX(cartesian_pose_msg->x);
    cartesian_pose_.setY(cartesian_pose_msg->y);
    cartesian_pose_.setZ(cartesian_pose_msg->z);
    cartesian_pose_.setTheta(cartesian_pose_msg->theta);
    cartesian_pose_.setQ5(cartesian_pose_msg->q5);
}
