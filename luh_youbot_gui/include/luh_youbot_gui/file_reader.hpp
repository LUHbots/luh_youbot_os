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
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/

#ifndef LUH_YOUBOT_GUI_FILE_READER_H
#define LUH_YOUBOT_GUI_FILE_READER_H

#include <luh_youbot_kinematics/arm_kinematics.h>
#include "common.hpp"

namespace luh_youbot_gui
{

class FileReader
{
public:
    FileReader();

    void setPosesFile(std::string filename);
    bool loadPoses(PoseMap &poses) const;
    void savePoses(const PoseMap &poses) const;
    std::string getFileName(){return poses_file_;}
    bool loadParameters(std::string filename);
    std::string getStringParam(std::string param_name);
    int getIntParam(std::string param_name);
    double getDoubleParam(std::string param_name);
    bool parametersLoaded(){return params_loaded_;}

private:
    std::string poses_file_;
    QMap<QString, QString> parameters_;
    bool params_loaded_;




};
}
#endif // FILE_READER_H
