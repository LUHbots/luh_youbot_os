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

#include "luh_youbot_gui/file_reader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace luh_youbot_gui;

//########## CONSTRUCTOR ###############################################################################################
FileReader::FileReader():
    params_loaded_(false)
{
}

//########## SET POSES FILE ############################################################################################
void FileReader::setPosesFile(std::string filename)
{
    poses_file_ = filename;
}

//########## LOAD POSES ################################################################################################
bool FileReader::loadPoses(PoseMap &poses) const
{
    try
    {

    std::vector<YAML::Node> nodes = YAML::LoadAllFromFile(poses_file_);
    if(nodes.empty())
        return false;

    for(uint i=0; i<nodes.size();i++)
    {
        QString name = QString::fromStdString(nodes[i].begin()->first.as<std::string>());
        YAML::Node joint_poses = nodes[i].begin()->second;

        luh_youbot_kinematics::JointPosition pose;

        for(YAML::const_iterator it=joint_poses.begin();it!=joint_poses.end();++it)
        {
            YAML::const_iterator map_it = it->begin();
            std::string joint_name = map_it->first.as<std::string>();
            double joint_value = map_it->second.as<double>();
            joint_value *= M_PI/180.0;

            if(!joint_name.compare("arm_joint_1"))
                pose.setQ1(joint_value);
            else if(!joint_name.compare("arm_joint_2"))
                pose.setQ2(joint_value);
            else if(!joint_name.compare("arm_joint_3"))
                pose.setQ3(joint_value);
            else if(!joint_name.compare("arm_joint_4"))
                pose.setQ4(joint_value);
            else if(!joint_name.compare("arm_joint_5"))
                pose.setQ5(joint_value);
        }

        pose.addOffset();
        poses[name] = pose;
    }

    }
    catch(YAML::Exception e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }

    return true;
}

//########## SAVE POSES ################################################################################################
void FileReader::savePoses(const PoseMap &poses) const
{
    std::cout << "Saving to " << poses_file_ << std::endl;
    std::ofstream fout(poses_file_.c_str());

    for(PoseMap::const_iterator it=poses.begin(); it != poses.end(); ++it)
    {
        fout << "---" << std::endl;

        luh_youbot_kinematics::JointPosition pose = *it;
        pose.subtractOffset();
        pose *= 180/M_PI;

        YAML::Node seq_node;
        seq_node[0]["arm_joint_1"] = pose.q1();
        seq_node[1]["arm_joint_2"] = pose.q2();
        seq_node[2]["arm_joint_3"] = pose.q3();
        seq_node[3]["arm_joint_4"] = pose.q4();
        seq_node[4]["arm_joint_5"] = pose.q5();

        YAML::Node map_node;
        map_node[it.key().toStdString()] = seq_node;

        fout << map_node << std::endl;
    }

}

//########## LOAD PARAMETERS ###########################################################################################
bool FileReader::loadParameters(std::string filename)
{
    try
    {
        YAML::Node node = YAML::LoadFile(filename);
        if(node.IsNull())
            return false;

        for(YAML::const_iterator it=node.begin();it!=node.end();++it)
        {
            QString param_name = QString::fromStdString(it->first.as<std::string>());
            QString value = QString::fromStdString(it->second.as<std::string>());

            parameters_[param_name] = value;
        }

    }
    catch(YAML::Exception e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }

    params_loaded_ = true;
    return true;
}

//########## GET STRING PARAM ##########################################################################################
std::string FileReader::getStringParam(std::string param_name)
{
    if(parameters_.find(QString::fromStdString(param_name)) != parameters_.end())
        return parameters_[QString::fromStdString(param_name)].toStdString();
    else
        return "";
}

//########## GET INT PARAM #############################################################################################
int FileReader::getIntParam(std::string param_name)
{
    if(parameters_.find(QString::fromStdString(param_name)) != parameters_.end())
        return parameters_[QString::fromStdString(param_name)].toInt();
    else
        return 0;
}

//########## GET DOUBLE PARAM ##########################################################################################
double FileReader::getDoubleParam(std::string param_name)
{
    if(parameters_.find(QString::fromStdString(param_name)) != parameters_.end())
    {
        return parameters_[QString::fromStdString(param_name)].toDouble();
    }
    else
    {
        return 0.0;
    }
}
