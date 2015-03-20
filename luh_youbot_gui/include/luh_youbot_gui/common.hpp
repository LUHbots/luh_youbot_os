/* *****************************************************************
 *
 * luh_youbot_gui
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
 * Author: Simon Aden (simon.aden@mailbox.org)
 ******************************************************************/

#ifndef LUH_YOUBOT_GUI_COMMON_HPP
#define LUH_YOUBOT_GUI_COMMON_HPP

#include <luh_youbot_kinematics/arm_kinematics.h>
#include <QString>
#include <QMap>

namespace luh_youbot_gui
{

typedef QMap<QString, luh_youbot_kinematics::JointPosition> PoseMap;
enum MoveMode{MODE_DIRECT, MODE_INTER, MODE_PLAN};
enum Coordinates{JOINTSPACE, CARTESIAN, CYLINDRIC};

}

#endif // LUH_YOUBOT_GUI_COMMON_HPP
