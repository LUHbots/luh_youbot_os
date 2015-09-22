#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <luh_youbot_msgs/SetGripperAction.h>

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    // === ROS INITIALISATION ===
    ros::init(argc, argv, "gripper_test_node");
    ros::NodeHandle node;

    if(argc != 2)
    {
        std::cout << "Usage: gripper_test_node position" << std::endl;
        return 0;
    }

    double position = atof(argv[1]);

    actionlib::SimpleActionClient<luh_youbot_msgs::SetGripperAction> ac("arm_1/set_gripper", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    luh_youbot_msgs::SetGripperGoal goal;
    goal.gripper_width = position;
    ac.sendGoal(goal);

    ROS_INFO("Goal sent.");

    ROS_INFO("Waiting for result...");
    ac.waitForResult();


    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
}
