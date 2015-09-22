#include "luh_youbot_gripper/gripper_node.h"

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    // === ROS INITIALISATION ===
    ros::init(argc, argv, "luh_gripper_node");
    ros::NodeHandle node;

    luh_youbot_gripper::GripperNode gripper(node);


    // === START LOOP ===
    ROS_INFO("Running...");

    ros::spin();

    return 0;
}
