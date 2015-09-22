/* *****************************************************************
 *
 * luh_youbot_controller
 *
 * Copyright (c) 2015,
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
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/

#include "luh_youbot_controller/node/controller_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");

    ros::NodeHandle node_handle;
    ControllerNode node(node_handle);

    ROS_INFO("Controller Node is spinning...");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
