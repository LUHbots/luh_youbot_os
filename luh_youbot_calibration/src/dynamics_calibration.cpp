#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <luh_youbot_controller_api/youbot_api.h>
#include <luh_youbot_kinematics/arm_dynamics.h>
#include <nlopt.hpp>

namespace yapi = youbot_api;
namespace ykin = luh_youbot_kinematics;

struct Data
{
    std::vector<std::vector<double> > positions;
    std::vector<std::vector<double> > velocities;
    std::vector<std::vector<double> > torques;
    std::vector<std::vector<double> > accelerations;
    std::vector<double> timestamps;
}data;

struct State
{
    ykin::JointPosition pos;
    ykin::JointVelocity vel;
    ykin::JointAcceleration acc;
    ykin::JointEffort trq;
}state;

yapi::YoubotArm arm;
unsigned long iteration_count;
ros::Time start_time;
ykin::YoubotArmDynamics dynamics;
double min_error = HUGE_VAL;
ykin::DynamicParameters best_params;

//############## JOINT STATE CALLBACK ##################################################################################
void jointStateCallback(const sensor_msgs::JointState::ConstPtr msg)
{
    for(uint i=0; i<5; i++)
    {
        data.positions[i].push_back(msg->position[i]);
        data.velocities[i].push_back(msg->velocity[i]);
        data.torques[i].push_back(msg->effort[i]);
    }
    data.timestamps.push_back((msg->header.stamp - start_time).toSec());
}

//############## WAIT ##################################################################################################
void wait()
{
    ros::Rate r(200);

    while(arm.isBusy())
    {
        ros::spinOnce();
        r.sleep();
    }
}

//############## GET INITIAL PARAMETERS ################################################################################
ykin::DynamicParameters getInitialParams()
{
    ykin::DynamicParameters params;

    params.friction_p_1[0]= 1.5;
    params.friction_p_1[1]= 0.6;
    params.friction_p_1[2]= 0.5;
    params.friction_p_1[3]= 0.2;
    params.friction_p_1[4]= 0.2;
    params.friction_p_2[0]= 100;
    params.friction_p_2[1]= 100;
    params.friction_p_2[2]= 100;
    params.friction_p_2[3]= 100;
    params.friction_p_2[4]= 100;
    params.friction_p_3[0]= 1.5;
    params.friction_p_3[1]= 1.0;
    params.friction_p_3[2]= 1.0;
    params.friction_p_3[3]= 0.1;
    params.friction_p_3[4]= 0.1;
    params.friction_n_1[0]= 1.0;
    params.friction_n_1[1]= 0.8;
    params.friction_n_1[2]= 0.5;
    params.friction_n_1[3]= 0.2;
    params.friction_n_1[4]= 0.2;
    params.friction_n_2[0]= 100;
    params.friction_n_2[1]= 100;
    params.friction_n_2[2]= 100;
    params.friction_n_2[3]= 100;
    params.friction_n_2[4]= 100;
    params.friction_n_3[0]= 1.5;
    params.friction_n_3[1]= 1.0;
    params.friction_n_3[2]= 1.0;
    params.friction_n_3[3]= 0.1;
    params.friction_n_3[4]= 0.1;
//    params.mass[0]        = ;
    params.mass[1]        = 1.155;
    params.mass[2]        = 0.934;
    params.mass[3]        = 0.877 + 0.251;
//    params.mass[4]        = ;
    params.inertia_x[0]   = 0.003863;
    params.inertia_x[1]   = 0.000823;
    params.inertia_x[2]   = 0.000535;
    params.inertia_x[3]   = 0.000869;
    params.inertia_x[4]   = 0.000119;
    params.inertia_y[1]   = 0.0044;
    params.inertia_y[2]   = 0.0025;
    params.inertia_y[3]   = 0.0011;
    params.inertia_y[4]   = 0.0003;
    params.inertia_z[1]   = 0.0044;
    params.inertia_z[2]   = 0.0025;
    params.inertia_z[3]   = 0.0011;
    params.inertia_z[4]   = 0.0003;
//    params.com_radius[0]  = ;
    params.com_radius[1]  = 0.5 * ykin::L2;
    params.com_radius[2]  = 0.5 * ykin::L3;
    params.com_radius[3]  = 0.3 * ykin::L4;
//    params.com_radius[4]  = ;
//    params.com_angle[0]   = 0.0;
    params.com_angle[1]   = 0.0;
    params.com_angle[2]   = 0.0;
    params.com_angle[3]   = 0.0;
//    params.com_angle[4]   = 0.0;
    params.g              = 9.81;

    return params;
}

//############## SERIALIZE PARAMETERS ##################################################################################
void serializeParameters(const ykin::DynamicParameters &params, std::vector<double> &x)
{
    x.resize(57);
    x[0]  = params.friction_p_1[0];
    x[1]  = params.friction_p_1[1];
    x[2]  = params.friction_p_1[2];
    x[3]  = params.friction_p_1[3];
    x[4]  = params.friction_p_1[4];
    x[5]  = params.friction_p_2[0];
    x[6]  = params.friction_p_2[1];
    x[7]  = params.friction_p_2[2];
    x[8]  = params.friction_p_2[3];
    x[9]  = params.friction_p_2[4];
    x[10] = params.friction_p_3[0];
    x[11] = params.friction_p_3[1];
    x[12] = params.friction_p_3[2];
    x[13] = params.friction_p_3[3];
    x[14] = params.friction_p_3[4];
    x[15] = params.friction_n_1[0];
    x[16] = params.friction_n_1[1];
    x[17] = params.friction_n_1[2];
    x[18] = params.friction_n_1[3];
    x[19] = params.friction_n_1[4];
    x[20] = params.friction_n_2[0];
    x[21] = params.friction_n_2[1];
    x[22] = params.friction_n_2[2];
    x[23] = params.friction_n_2[3];
    x[24] = params.friction_n_2[4];
    x[25] = params.friction_n_3[0];
    x[26] = params.friction_n_3[1];
    x[27] = params.friction_n_3[2];
    x[28] = params.friction_n_3[3];
    x[29] = params.friction_n_3[4];
    x[30] = params.mass[0];
    x[31] = params.mass[1];
    x[32] = params.mass[2];
    x[33] = params.mass[3];
    x[34] = params.mass[4];
    x[35] = params.inertia_x[0];
    x[36] = params.inertia_x[1];
    x[37] = params.inertia_x[2];
    x[38] = params.inertia_x[3];
    x[39] = params.inertia_x[4];
    x[40] = params.inertia_y[1];
    x[41] = params.inertia_y[2];
    x[42] = params.inertia_y[3];
    x[43] = params.inertia_z[1];
    x[44] = params.inertia_z[2];
    x[45] = params.inertia_z[3];
    x[46] = params.com_radius[0];
    x[47] = params.com_radius[1];
    x[48] = params.com_radius[2];
    x[49] = params.com_radius[3];
    x[50] = params.com_radius[4];
    x[51] = params.com_angle[0];
    x[52] = params.com_angle[1];
    x[53] = params.com_angle[2];
    x[54] = params.com_angle[3];
    x[55] = params.com_angle[4];
    x[56] = params.g;
}

//############## DESERIALIZE PARAMETERS ################################################################################
void deserializeParameters(const std::vector<double> &x, ykin::DynamicParameters &params)
{
     params.friction_p_1[0]= x[0] ;
     params.friction_p_1[1]= x[1] ;
     params.friction_p_1[2]= x[2] ;
     params.friction_p_1[3]= x[3] ;
     params.friction_p_1[4]= x[4] ;
     params.friction_p_2[0]= x[5] ;
     params.friction_p_2[1]= x[6] ;
     params.friction_p_2[2]= x[7] ;
     params.friction_p_2[3]= x[8] ;
     params.friction_p_2[4]= x[9] ;
     params.friction_p_3[0]= x[10];
     params.friction_p_3[1]= x[11];
     params.friction_p_3[2]= x[12];
     params.friction_p_3[3]= x[13];
     params.friction_p_3[4]= x[14];
     params.friction_n_1[0]= x[15];
     params.friction_n_1[1]= x[16];
     params.friction_n_1[2]= x[17];
     params.friction_n_1[3]= x[18];
     params.friction_n_1[4]= x[19];
     params.friction_n_2[0]= x[20];
     params.friction_n_2[1]= x[21];
     params.friction_n_2[2]= x[22];
     params.friction_n_2[3]= x[23];
     params.friction_n_2[4]= x[24];
     params.friction_n_3[0]= x[25];
     params.friction_n_3[1]= x[26];
     params.friction_n_3[2]= x[27];
     params.friction_n_3[3]= x[28];
     params.friction_n_3[4]= x[29];
     params.mass[0]        = x[30];
     params.mass[1]        = x[31];
     params.mass[2]        = x[32];
     params.mass[3]        = x[33];
     params.mass[4]        = x[34];
     params.inertia_x[0]   = x[35];
     params.inertia_x[1]   = x[36];
     params.inertia_x[2]   = x[37];
     params.inertia_x[3]   = x[38];
     params.inertia_x[4]   = x[39];
     params.inertia_y[1]   = x[40];
     params.inertia_y[2]   = x[41];
     params.inertia_y[3]   = x[42];
     params.inertia_z[1]   = x[43];
     params.inertia_z[2]   = x[44];
     params.inertia_z[3]   = x[45];
     params.com_radius[0]  = x[46];
     params.com_radius[1]  = x[47];
     params.com_radius[2]  = x[48];
     params.com_radius[3]  = x[49];
     params.com_radius[4]  = x[50];
     params.com_angle[0]   = x[51];
     params.com_angle[1]   = x[52];
     params.com_angle[2]   = x[53];
     params.com_angle[3]   = x[54];
     params.com_angle[4]   = x[55];
     params.g              = x[56];
}

//############## OPTIMIZATION FUNCTION #################################################################################
double errorFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
//    Data *data = reinterpret_cast<Data*>(func_data);
    ykin::DynamicParameters p;
    deserializeParameters(x, p);
    dynamics.setParameters(p);

    double error = 0;

    for(uint i=0; i<data.positions.front().size(); i++)
    {
        for(uint j=0; j<5;j++)
        {
            state.pos[j] = data.positions[j][i];
            state.vel[j] = data.velocities[j][i];
            state.acc[j] = data.accelerations[j][i];
            state.trq[i] = data.torques[j][i];
        }

        dynamics.setJointPosition(state.pos);
        dynamics.setJointVelocity(state.vel);
        dynamics.setJointAcceleration(state.acc);

        ykin::JointEffort trq_diff = dynamics.getJointEffort() - state.trq;

        for(uint j=0; j<5;j++)
        {
            error += trq_diff[j]*trq_diff[j];
        }
    }

    if(error < min_error)
    {
        min_error = error;
        best_params = p;
    }

    if(iteration_count % 100000 == 0 && iteration_count > 0)
        ROS_INFO("Iterations: %lu; Min. error: %f", iteration_count, min_error);

    iteration_count++;

    return error;
}

//############## EVALUATE ##############################################################################################
void evaluate()
{
    int n_params = 57;

    ROS_INFO("Evaluating...");

    // === INITIALIZE OPTIMIZER ===
    //        nlopt::opt opt(nlopt::GN_ESCH , n_params);
    nlopt::opt opt(nlopt::LN_COBYLA, n_params);

    // lower bounds
    std::vector<double> lb(n_params, -10);// TODO
    opt.set_lower_bounds(lb);

    // upper bounds
    std::vector<double> lu(n_params, 2000);// TODO
    opt.set_upper_bounds(lu);

    // termination criterion
    opt.set_xtol_rel(1e-6);

    // function to minimize
    opt.set_min_objective(errorFunction, &data);

    // initial values
    ykin::DynamicParameters params = getInitialParams();

    std::vector<double> x(n_params, 0);
    serializeParameters(params, x);
    iteration_count = 0;

    // === START OPTIMIZATION ===
    double minf;
    nlopt::result result = opt.optimize(x, minf);

    if(result < 0)
    {
        ROS_ERROR("Optimization failed.");
        return;
    }

    return;
}

//############## MAIN ##################################################################################################
int main(int argc, char** argv)
{
    // === INITIALISATION ===
    ros::init(argc, argv, "dynamics_calibration");
    ros::NodeHandle node;
    arm.init(node);
    ros::Subscriber joint_state_sub = node.subscribe("arm_1/joint_states", 1000, jointStateCallback);
    data.positions.resize(5);
    data.velocities.resize(5);
    data.torques.resize(5);
    data.accelerations.resize(5);
    start_time = ros::Time::now();

    // === DO CALIBRATION MOVEMENT ===

    yapi::JointPath path;
    ykin::JointPosition pos;

    // joint 1
    pos.setQ1(-160.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ1(160.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ1(0);
    path.push_back(pos);

    // joint 2
    pos.setQ2(-60.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ2(80.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ2(0);
    path.push_back(pos);

    // joint 3
    pos.setQ3(-140.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ3(140.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ3(0);
    path.push_back(pos);

    // joint 4
    pos.setQ4(-100.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ4(90.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ4(0);
    path.push_back(pos);

    // joint 5
    pos.setQ5(-160.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ5(160.0/180.0*M_PI);
    path.push_back(pos);
    pos.setQ5(0);
    path.push_back(pos);

    // all at once
    ykin::CylindricPosition grip_pos;
    grip_pos.setQ1(33.33/180.0*M_PI);
    grip_pos.setR(0.2);
    grip_pos.setZ(0);
    grip_pos.setTheta(M_PI);
    grip_pos.setQ5(-M_PI/2);

    path.push_back(grip_pos.toJointspace(false));

    grip_pos.setQ1(-146.67);
    grip_pos.setR(-0.2);
    grip_pos.setZ(0);
    grip_pos.setTheta(-M_PI);
    grip_pos.setQ5(M_PI/2);

    path.push_back(grip_pos.toJointspace(true));

    grip_pos.setQ1(33.33/180.0*M_PI);
    grip_pos.setR(0.2);
    grip_pos.setZ(0);
    grip_pos.setTheta(M_PI);
    grip_pos.setQ5(-M_PI/2);

    path.push_back(grip_pos.toJointspace(false));

    pos = ykin::JointPosition();
    pos.setQ2(80.0/180.0*M_PI);
    path.push_back(pos);

    arm.moveAlongPath(path);
    wait();

    // hold position
    ros::Time t = ros::Time::now();
    double d = 0;
    ros::Rate r(200);
    while(d < 2.0)
    {
        ros::spinOnce();
        r.sleep();
        d = (ros::Time::now() - t).toSec();
    }

    path.clear();
    pos.setQ1(-M_PI/2);
    path.push_back(pos);
    pos.setQ1(M_PI/2);
    path.push_back(pos);

    arm.moveAlongPath(path);
    wait();

    joint_state_sub.shutdown();

    // === FILTER RESULTS ===
    int w = 3;
    std::vector<double> window;
    window.push_back(0.006);
    window.push_back(0.061);
    window.push_back(0.242);
    window.push_back(0.383);
    window.push_back(0.242);
    window.push_back(0.061);
    window.push_back(0.006);

    for(uint i=0; i<5; i++)
    {
        for(uint j=w; j<data.positions.front().size()-w-1; j++)
        {
            double p_mean = 0;
            double v_mean = 0;
            double t_mean = 0;
            for(int k=-w; k<=w; k++)
            {
                p_mean += data.positions[i][j+k] * window[w+k];
                v_mean += data.velocities[i][k] * window[w+k];
                t_mean += data.torques[i][k] * window[w+k];
            }

            data.positions[i][j] = p_mean;
            data.velocities[i][j] = p_mean;
            data.torques[i][j] = p_mean;
        }
    }

    // === CALCULATE ACCELERATION ===

    for(uint i=0; i<5; i++)
    {
        data.accelerations[i].resize(data.positions[i].size());
        for(uint j=1; j<data.positions.front().size()-1; j++)
        {
            data.accelerations[i][j] = (data.velocities[i][j+1] - data.velocities[i][j-1])
                    /(data.timestamps[j+1] - data.timestamps[j-1]);
        }

        data.accelerations[i][0] = data.accelerations[i][1];
        data.accelerations[i].back() = data.accelerations[i][data.accelerations[i].size()-2];
    }

    // === EVALUATE RESULTS ===
    evaluate();

    return 0;
}
