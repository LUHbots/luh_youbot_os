/* *****************************************************************
 *
 * luh_youbot_poses
 *
 * Copyright (c) 2014,
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
 ******************************************************************/

#ifndef LUH_YOUBOT_POSES_H
#define LUH_YOUBOT_POSES_H

#include <yaml-cpp/yaml.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <ros/package.h>
#include <ros/ros.h>

namespace youbot_poses
{

typedef std::map<std::string, luh_youbot_kinematics::JointPosition> PoseMap;

PoseMap read(std::string filename = "poses.yaml");

}

#endif // LUH_YOUBOT_POSES_H
