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

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include <luh_youbot_controller_api/youbot_api.h>
#include <luh_youbot_controller_api/controller_api.h>
//#include <luh_youbot_kinematics/arm_dynamics.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <nlopt.hpp>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <luh_youbot_kinematics/arm_dynamics.h>


namespace ykin = luh_youbot_kinematics;
struct State
{
    ykin::JointPosition pos;
    ykin::JointVelocity vel;
    ykin::JointVector trq;
};


std::vector<State> data;

double gravity = 9.81;
ykin::StaticParameters best_params;
int msg_counter;

unsigned long iteration_count;
double min_error = HUGE_VAL;

std::vector<std::string> param_names;

bool save_data;
std::string datafile;
std::string paramfile;

ykin::YoubotArmDynamics dynamics;

// DECLARATIONS FOR AUTOMATED CALIBRATION MOVEMENT
std::string calibrationPosesFilePath;
youbot_api::YoubotArm arm;
std::vector<luh_youbot_kinematics::JointPosition> posesMatrix;
luh_youbot_kinematics::JointPosition poseRow;

//############## JOINT STATE CALLBACK ##################################################################################
void jointStateCallback(const sensor_msgs::JointState::ConstPtr msg)
{
    if(save_data)
    {
        ykin::JointPosition jnt_pos(msg->position);
        jnt_pos.addOffset();
        data.back().pos += jnt_pos;
        data.back().vel += ykin::JointVelocity(msg->velocity);
        data.back().trq += ykin::JointVector(msg->effort);

        msg_counter++;
    }
}

//############## SERIALIZE PARAMETERS ##################################################################################
void serializeParameters(const ykin::StaticParameters &params, std::vector<double> &x)
{
    x.resize(16);
    x[0]  = params.mass_5;
    x[1]  = params.mass_4;
    x[2]  = params.mass_3;
    x[3]  = params.mass_2;
    x[4]  = params.com_radius_5;
    x[5]  = params.com_radius_4;
    x[6]  = params.com_radius_3;
    x[7]  = params.com_radius_2;
    x[8]  = params.com_angle_5;
    x[9]  = params.com_angle_4;
    x[10]  = params.com_angle_3;
    x[11]  = params.com_angle_2;
    x[12]  = params.friction_5;
    x[13]  = params.friction_4;
    x[14] = params.friction_3;
    x[15] = params.friction_2;


}

//############## DESERIALIZE PARAMETERS ################################################################################
void deserializeParameters(const std::vector<double> &x, ykin::StaticParameters &params)
{
    params.mass_5         = x[0];
    params.mass_4         = x[1];
    params.mass_3         = x[2];
    params.mass_2         = x[3];
    params.com_radius_5   = x[4];
    params.com_radius_4   = x[5];
    params.com_radius_3   = x[6];
    params.com_radius_2   = x[7];
    params.com_angle_5    = x[8];
    params.com_angle_4    = x[9];
    params.com_angle_3    = x[10];
    params.com_angle_2    = x[11];
    params.friction_5     = x[12];
    params.friction_4     = x[13];
    params.friction_3     = x[14];
    params.friction_2     = x[15];
    //    params.gravity        = x[9];

}

//############## OPTIMIZATION FUNCTION #################################################################################
double errorFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
    //    Data *data = reinterpret_cast<Data*>(func_data);
    ykin::StaticParameters p;
    deserializeParameters(x, p);

    p.gravity = 9.81;
    dynamics.setStaticParameters(p);

    double error = 0;

    for(uint i=0; i<data.size(); i++)
    {
        ykin::JointVector trq_diff = dynamics.getStaticJointEffort(data[i].pos) - data[i].trq;

        for(uint j=1; j<5;j++)
        {
            error += trq_diff[j]*trq_diff[j];
        }
    }

    if(error < min_error)
    {
        min_error = error;
        best_params = p;
    }

    if(iteration_count % 10000 == 0 && iteration_count > 0)
    {
        ROS_INFO("Iterations: %lu; Min. error: %f", iteration_count, min_error);
        ROS_INFO("Best result so far:");
        for(uint i=0; i<param_names.size(); i++)
        {
            ROS_INFO("%s: %f", param_names[i].c_str(), x[i]);
        }
    }

    iteration_count++;

    return error;
}

//############## EVALUATE ##############################################################################################
void evaluate()
{
    int n_params = 16;

    ROS_INFO("Evaluating...");

    // === INITIALIZE OPTIMIZER ===
    nlopt::opt opt(nlopt::GN_ESCH , n_params);
//    nlopt::opt opt(nlopt::LN_COBYLA, n_params);

    // initial values
    ykin::StaticParameters init_params;
    init_params.mass_5 = 0.251;
    init_params.mass_4 = 0.877;
    init_params.mass_3 = 0.934;
    init_params.mass_2 = 1.155;
    init_params.com_radius_5 = 0.8 * ykin::L4;
    init_params.com_radius_4 = 0.2 * ykin::L4;
    init_params.com_radius_3 = 0.5 * ykin::L3;
    init_params.com_radius_2 = 0.5 * ykin::L2;
    init_params.com_angle_5 = 0.0;
    init_params.com_angle_4 = 0.0;
    init_params.com_angle_3 = 0.0;
    init_params.com_angle_2 = 0.0;
    init_params.friction_2 = 0.0;
    init_params.friction_3 = 0.0;
    init_params.friction_4 = 0.0;
    init_params.friction_5 = 0.0;
    //    init_params.gravity = 9.81;

    // lower bounds
    ykin::StaticParameters lower_bound;
    lower_bound.mass_5 = 0.0;
    lower_bound.mass_4 = 0.0;
    lower_bound.mass_3 = 0.0;
    lower_bound.mass_2 = 0.0;
    lower_bound.com_radius_5 = 0.0;
    lower_bound.com_radius_4 = 0.0;
    lower_bound.com_radius_3 = 0.0;
    lower_bound.com_radius_2 = 0.0;
    lower_bound.com_angle_5 = -M_PI/2;
    lower_bound.com_angle_4 = -M_PI/2;
    lower_bound.com_angle_3 = -M_PI/2;
    lower_bound.com_angle_2 = -M_PI/2;
    lower_bound.friction_2 = -5.0;
    lower_bound.friction_3 = -5.0;
    lower_bound.friction_4 = -5.0;
    lower_bound.friction_5 = -5.0;
    //    lower_bound.gravity = 8.0;

    std::vector<double> lb;
    serializeParameters(lower_bound, lb);
    opt.set_lower_bounds(lb);

    // upper bounds
    ykin::StaticParameters upper_bound;
    upper_bound.mass_5 = 5;
    upper_bound.mass_4 = 5;
    upper_bound.mass_3 = 5;
    upper_bound.mass_2 = 5;
    upper_bound.com_radius_5 = ykin::L4;
    upper_bound.com_radius_4 = ykin::L4;
    upper_bound.com_radius_3 = ykin::L3;
    upper_bound.com_radius_2 = ykin::L2;
    upper_bound.com_angle_5 = M_PI;
    upper_bound.com_angle_4 = M_PI;
    upper_bound.com_angle_3 = M_PI;
    upper_bound.com_angle_2 = M_PI;
    upper_bound.friction_2 = 5.0;
    upper_bound.friction_3 = 5.0;
    upper_bound.friction_4 = 5.0;
    upper_bound.friction_5 = 5.0;
    //    upper_bound.gravity = 12;

    std::vector<double> ub;
    serializeParameters(upper_bound, ub);
    opt.set_upper_bounds(ub);

    // termination criterion
    opt.set_xtol_rel(1e-12);

    // function to minimize
    opt.set_min_objective(errorFunction, &data);

    // initial values
    std::vector<double> x;
    serializeParameters(init_params, x);
    iteration_count = 0;

    // === START OPTIMIZATION ===
    double minf;
    nlopt::result result = opt.optimize(x, minf);

    if(result < 0)
    {
        ROS_ERROR("Optimization failed.");
        return;
    }

    dynamics.setStaticParameters(best_params);
    for(uint i=0; i<data.size(); i++)
    {
        std::cout << "--------- POSE " << i << " ----------" << std::endl;

        dynamics.getStaticJointEffort(data[i].pos).printValues("Calculated torques");
        data[i].trq.printValues("Measured Torques");
    }

    ROS_INFO("Optimization finished after %lu iterations (min. error: %f).", iteration_count, min_error);
    ROS_INFO("The result is:");
    for(uint i=0; i<param_names.size(); i++)
    {
        ROS_INFO("%s: %f", param_names[i].c_str(), x[i]);
    }



    return;
}

//############## LOAD FILE #############################################################################################
void load()
{
    YAML::Node node = YAML::LoadFile(datafile);
    data.resize(node.size());

    for (uint i=0;i<node.size();i++)
    {
        for(uint j=0; j<5; j++)
        {
            data[i].pos[j] = node[i]["position"][j].as<double>();
            data[i].trq[j] = node[i]["torque"][j].as<double>();
        }
    }

    std::cout << data.size() << " poses loaded." << std::endl;
}

//############## SAVE FILE #############################################################################################
void saveParams()
{
    YAML::Node out;
    std::vector<double> p;
    serializeParameters(best_params, p);

    for(uint i=0; i<param_names.size(); i++)
    {
        out[param_names[i]] = p[i];
    }
    out["gravity"] = gravity;

    std::ofstream fout(paramfile.c_str());
    fout << out;
    std::cout << "Result saved in " << paramfile << std::endl;
}

//############## SAVE FILE #############################################################################################
void saveData()
{
    YAML::Node out;

    for(uint i=0; i<data.size(); i++)
    {
        for(uint j=0; j<5; j++)
        {
            out[i]["position"][j] = data[i].pos[j];
            out[i]["torque"][j] = data[i].trq[j];
        }
    }

    std::ofstream fout(datafile.c_str());
    fout << out;
    std::cout << "File saved." << std::endl;
}

//############## MAIN ##################################################################################################
int main(int argc, char** argv)
{
    // === INITIALISATION ===
    ros::init(argc, argv, "static_calibration");
    ros::NodeHandle node;
    ros::Subscriber joint_state_sub = node.subscribe("arm_1/joint_states", 1000, jointStateCallback);
    arm.init(node);

    param_names.push_back("mass_5");
    param_names.push_back("mass_4");
    param_names.push_back("mass_3");
    param_names.push_back("mass_2");
    param_names.push_back("com_radius_5");
    param_names.push_back("com_radius_4");
    param_names.push_back("com_radius_3");
    param_names.push_back("com_radius_2");
    param_names.push_back("com_angle_5");
    param_names.push_back("com_angle_4");
    param_names.push_back("com_angle_3");
    param_names.push_back("com_angle_2");
    param_names.push_back("friction_5");
    param_names.push_back("friction_4");
    param_names.push_back("friction_3");
    param_names.push_back("friction_2");

    //    param_names.push_back("gravity");

    datafile = ros::package::getPath("luh_youbot_calibration");
    paramfile = datafile;
    calibrationPosesFilePath = datafile;///
    datafile.append("/data/data.yaml");
    paramfile.append("/data/params.yaml");
    calibrationPosesFilePath.append("/data/calibrationPoses.yaml"); ///

    save_data = false;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::cout << "Load file? [y/n]" << std::endl;
    std::string ans;
    std::cin >> ans;
    if(ans[0] == 'y')
        load();

    std::cout << "automated or manuel pose adoption? [a/m]" << std::endl;
    std::cin >> ans;
    std::ifstream posesFile;
    ykin::JointPosition pose;

    posesFile.open(calibrationPosesFilePath.c_str());
    ros::Rate r(200);

    // === DO (MANUEL) CALIBRATION MOVEMENT ===
    if(ans[0] != 'a')
    {
        while(true)
        {
            std::cout << "Move the arm then type 's' to save pose or 'q' to quit." << std::endl;
            std::string ans;
            std::cin >> ans;

            if(ans[0] == 'q')
                break;

            if(ans[0] != 's')
                continue;

            // get 40 messages
            data.push_back(State());
            msg_counter = 0;
            save_data = true;
            std::cout << "Saving joint states..." << std::endl;

            while(msg_counter < 40)
            {
                r.sleep();
            }

            save_data = false;

            data.back().pos /= msg_counter;
            data.back().vel /= msg_counter;
            data.back().trq /= msg_counter;

            std::cout << data.size() << " poses saved." << std::endl;
            data.back().trq.printValues("Torqes");
        }
    }
    // === DO (AUTOMATED) CALIBRATION MOVEMENT ===
    else
    {
        if (posesFile.is_open())
        {
            //count number of poses in posesFile
            std::string line;
            int totalPoseNumber = -1;
            while(std::getline(posesFile, line))
            {
                if (line == "---")
                    totalPoseNumber++;
            }
            std::cout << "totalPoseNumber (starting with 0) in " << calibrationPosesFilePath.c_str() << " is: " << totalPoseNumber << std::endl;

            //Write values q1, q2, q, q4, q5 of poseFile in poseMatrix

            int poseNumber = -1;
            posesFile.clear();
            posesFile.seekg(0, std::ios::beg);

            char buffer [100];
            while(std::getline(posesFile, line))
            {
                if (line == "---")
                {
                    poseNumber++;

                    std::getline(posesFile, line); //Pose Name

                    // q1
                    std::getline(posesFile, line, ':');
                    std::getline(posesFile, line);
                    strcpy(buffer,line.c_str());
                    poseRow [0] = atof (buffer);


                    // q2
                    std::getline(posesFile, line, ':');
                    std::getline(posesFile, line);
                    strcpy(buffer,line.c_str());
                    poseRow [1] = atof (buffer);


                    // q3
                    std::getline(posesFile, line, ':');
                    std::getline(posesFile, line);
                    strcpy(buffer,line.c_str());
                    poseRow [2] = atof (buffer);

                    // q4
                    std::getline(posesFile, line, ':');
                    std::getline(posesFile, line);
                    strcpy(buffer,line.c_str());
                    poseRow [3] = atof (buffer);

                    // q5
                    std::getline(posesFile, line, ':');
                    std::getline(posesFile, line);
                    strcpy(buffer,line.c_str());
                    poseRow [4] = atof (buffer);

                    posesMatrix.push_back(poseRow);
                }
            }

            std::cout << "Type 's' to initialize each movement to the next pose via console input or 'a' to use the automated mode." << std::endl;
            std::string ans_initMovementManuel;
            std::cin >> ans_initMovementManuel;
            //Move to positions from posesMatrix
            for (uint i = 0; i < totalPoseNumber + 1; i++)
            {
                //set joint values
                pose.setQ1( posesMatrix [i] [0]);
                pose.setQ2( posesMatrix [i] [1]);
                pose.setQ3( posesMatrix [i] [2]);
                pose.setQ4( posesMatrix [i] [3]);
                pose.setQ5( posesMatrix [i] [4]);

                if (ans_initMovementManuel[0] != 'a')
                {
                    std::cout << "Type 's' to start the movement to the next pose and to save pose or 'q' to quit." << std::endl;
                    std::string ans;
                    std::cin >> ans;

                    if(ans[0] == 'q')
                        break;

                    if(ans[0] != 's')
                        continue;
                }

                pose *= M_PI/180.0;
                pose.addOffset();
                arm.moveToPose(pose);
                arm.waitForCurrentAction();
                ros::Duration(0.9).sleep(); // sleep for half a second

                // get 40 messages
                data.push_back(State());
                msg_counter = 0;
                save_data = true;
                std::cout << "Saving joint states..." << std::endl;

                while(msg_counter < 120)
                {
                    r.sleep();
                }

                save_data = false;

                data.back().pos /= msg_counter;
                data.back().vel /= msg_counter;
                data.back().trq /= msg_counter;

                std::cout << data.size() << " poses saved." << std::endl;
                data.back().trq.printValues("Torqes");
            }
        }
        else
        {
            std::cout << "failed to open: " << calibrationPosesFilePath.c_str() << std::endl;
        }
    }

    // === EVALUATE RESULTS ===
    std::cout << "Save captured data? [y/n]" << std::endl;
    std::cin >> ans;
    if(ans[0] == 'y')
        saveData();

    std::cout << "Evaluate? [y/n]" << std::endl;
    std::cin >> ans;
    if(ans[0] == 'y')
        evaluate();

    std::cout << "Save result? [y/n]" << std::endl;
    std::cin >> ans;
    if(ans[0] == 'y')
        saveParams();

    return 0;
}
