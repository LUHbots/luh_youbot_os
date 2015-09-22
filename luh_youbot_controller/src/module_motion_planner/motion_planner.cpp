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

#include "luh_youbot_controller/module_motion_planner/motion_planner.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "ros/package.h"

using namespace luh_youbot_kinematics;

//############### POSE SET CONSTRUCTOR #################################################################################
PoseSet::PoseSet()
{

}

//############### GET MIN COST ELEMENT #################################################################################
Pose* PoseSet::getMinCostPose()
{
    Pose* min_element = NULL;
    double min_cost = 1e6;
    for(uint i=0; i<this->size(); i++)
    {
        if(this->data()[i]->status == CLOSED)
            continue;

        double total_cost = this->data()[i]->cost + this->data()[i]->estimate;
        if(total_cost < min_cost)
        {
            min_cost = total_cost;
            min_element = this->data()[i];
        }
    }
    return min_element;
}

//############### MOTION PLANNER CONSTRUCTOR ###########################################################################
ModuleMotionPlanner::ModuleMotionPlanner(): ControllerModule(),
    controller_(NULL),
    joint_path_server_(NULL),
    cartesian_path_server_(NULL),
    cylindric_path_server_(NULL),
    named_path_server_(NULL),
    cartesian_pose_server_(NULL),
    cylindric_pose_server_(NULL),
    joint_pose_server_(NULL),
    named_pose_server_(NULL),
    torque_controller_(NULL),
    youbot_dynamics_(NULL)
{

}

//############### MOTION PLANNER DESTRUCTOR ############################################################################
ModuleMotionPlanner::~ModuleMotionPlanner()
{
    delete controller_;
    delete joint_path_server_;
    delete cartesian_path_server_;
    delete cylindric_path_server_;
    delete named_path_server_;
    delete cartesian_pose_server_;
    delete cylindric_pose_server_;
    delete joint_pose_server_;
    delete named_pose_server_;
    delete torque_controller_;
    delete youbot_dynamics_;
}

//############### INITIALISATION #######################################################################################
void ModuleMotionPlanner::init()
{    
    ROS_INFO("MMP: Initialising Motion Planner Module...");

    // === PARAMS ===
    std::string posefile;
    std::string neighborfile;
    node_->param("module_motion_planner/poses_file", posefile, std::string("cfg/module_motion_planner/poses.yaml"));
    node_->param("module_motion_planner/neighbors_file", neighborfile,
                 std::string("cfg/module_motion_planner/neighbors.yaml"));
    node_->param("module_motion_planner/euclidean_influence_factor", euclidean_influence_factor_, 1.0);
    node_->param("module_motion_planner/constant_influence", constant_influence_, 0.1);
    node_->param("module_motion_planner/velocity_factor", velocity_factor_, 0.6);    
    double goal_tolerance, goal_percentage;
    node_->param("module_motion_planner/goal_tolerance", goal_tolerance, 30.0);
    node_->param("module_motion_planner/tolerance_check_percentage", goal_percentage, 0.8);
    node_->param("module_motion_planner/finepos_pos_tolerance", finepos_pos_tolerance_, 1.0);
    finepos_pos_tolerance_ *= M_PI / 180.0;
    node_->param("module_motion_planner/finepos_vel_tolerance", finepos_vel_tolerance_, 0.1);
    finepos_vel_tolerance_ *= M_PI / 180.0;
    double q1_w, q2_w, q3_w, q4_w, q5_w;
    node_->param("module_motion_planner/q1_weight", q1_w, 1.0);
    node_->param("module_motion_planner/q2_weight", q2_w, 1.0);
    node_->param("module_motion_planner/q3_weight", q3_w, 1.0);
    node_->param("module_motion_planner/q4_weight", q4_w, 1.0);
    node_->param("module_motion_planner/q5_weight", q5_w, 1.0);
    joint_weights_.setQ1(q1_w);
    joint_weights_.setQ2(q2_w);
    joint_weights_.setQ3(q3_w);
    joint_weights_.setQ4(q4_w);
    joint_weights_.setQ5(q5_w);


    std::string mode_string;
    node_->param("module_motion_planner/command_mode", mode_string, std::string("POSITION"));
    if(mode_string.compare("VELOCITY") == 0)
    {
        ROS_INFO("MMP: Running in VELOCITY CONTROL MODE");
        command_mode_ = VELOCITY;
    }
    else if(mode_string.compare("TORQUE") == 0)
    {
        ROS_INFO("MMP: Running in TORQUE CONTROL MODE");
        command_mode_ = TORQUE;
    }
    else
    {
        ROS_INFO("MMP: Running in POSITION CONTROL MODE");
        command_mode_ = POSITION;
    }

    deactivated = false;

    joint_velocities_.setQ1(1.0); // todo: sinnvolle werte?
    joint_velocities_.setQ2(1.4);
    joint_velocities_.setQ3(1.6);
    joint_velocities_.setQ4(1.8);
    joint_velocities_.setQ5(1.8);


    // === INITIALISE POSE GRAPH ===
    if(!load(posefile, neighborfile))
    {
        ROS_ERROR("MMP: Initialisation failed.");
        this->status_ = STATUS_FAILURE;
        return;
    }
    else
    {
        // === MOTION CONTROLLER ===
        controller_ = new motionPlanning();
        controller_->setUpdateFrequency(arm_update_frequency_);
        controller_->setGoalTolerance(goal_tolerance * M_PI/180.0, goal_percentage);

        // === SERVICE SERVERS ===
        predef_pose_server_ = node_->advertiseService("arm_1/to_predefined_pose",
                                                      &ModuleMotionPlanner::predefPoseCallback, this);
        list_poses_server_ = node_->advertiseService("arm_1/list_poses", &ModuleMotionPlanner::listPosesCallback, this);

        // === ACTION SERVER ===

        //pose commands
        cartesian_pose_server_ = new CartesianPoseServer(*node_, "arm_1/to_cartesian_pose/plan", false);
        cylindric_pose_server_ = new CylindricPoseServer(*node_, "arm_1/to_cylindric_pose/plan", false);
        joint_pose_server_     = new JointPoseServer(*node_, "arm_1/to_joint_pose/plan", false);
        named_pose_server_     = new NamedPoseServer(*node_, "arm_1/to_named_pose/plan", false);

        cartesian_pose_server_->registerGoalCallback(boost::bind(&ModuleMotionPlanner::cartesianPoseCallback, this));
        cylindric_pose_server_->registerGoalCallback(boost::bind(&ModuleMotionPlanner::cylindricPoseCallback, this));
        joint_pose_server_    ->registerGoalCallback(boost::bind(&ModuleMotionPlanner::jointPoseCallback, this));
        named_pose_server_    ->registerGoalCallback(boost::bind(&ModuleMotionPlanner::namedPoseCallback, this));

        cartesian_pose_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));
        cylindric_pose_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));
        joint_pose_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));
        named_pose_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));

        //path commands
        cartesian_path_server_ = new CartesianPathServer(*node_, "arm_1/cartesian_path", false);
        cylindric_path_server_ = new CylindricPathServer(*node_, "arm_1/cylindric_path", false);
        joint_path_server_     = new JointPathServer(*node_, "arm_1/joint_path", false);
        named_path_server_     = new NamedPathServer(*node_, "arm_1/named_path", false);

        cartesian_path_server_->registerGoalCallback(
                    boost::bind(&ModuleMotionPlanner::cartesianPathCallback, this));
        cylindric_path_server_->registerGoalCallback(
                    boost::bind(&ModuleMotionPlanner::cylindricPathCallback, this));
        joint_path_server_    ->registerGoalCallback(
                    boost::bind(&ModuleMotionPlanner::jointPathCallback, this));
        named_path_server_    ->registerGoalCallback(
                    boost::bind(&ModuleMotionPlanner::namedPathCallback, this));

        cartesian_path_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));
        cylindric_path_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));
        joint_path_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));
        named_path_server_->registerPreemptCallback(boost::bind(&ModuleMotionPlanner::preemptCallback, this));

        // start servers
        cartesian_pose_server_->start();
        cylindric_pose_server_->start();
        joint_pose_server_->start();
        named_pose_server_->start();
        cartesian_path_server_->start();
        cylindric_path_server_->start();
        joint_path_server_->start();
        named_path_server_->start();

        // === PUBLISHER ===
        //        position_error_publisher_ =
        //                node_->advertise<luh_youbot_msgs::JointVector>("motion_planner/position_error", 100);

        ROS_INFO("MMP: Motion Planner Module initialised.");
    }
}

//############### UPDATE ###############################################################################################
void ModuleMotionPlanner::update()
{
    if(status_ != STATUS_ACTIVE)
        return;

    JointPosition joint_position = youbot_->arm()->getJointPosition();

    controller_->update(joint_position);

    if(command_mode_ == POSITION)
    {
        joint_position = controller_->getPosition();
        youbot_->arm()->setJointPositions(joint_position);
    }
    else if(command_mode_ == VELOCITY)
    {
        JointVelocity joint_velocity = controller_->getVelocity();
        youbot_->arm()->setJointVelocities(joint_velocity);
    }
    else if(command_mode_ == TORQUE)
    {
        ros::Time time_now = ros::Time::now();
        double dt = (time_now - last_update_time_).toSec();
        last_update_time_ = time_now;

        JointVelocity joint_velocity = controller_->getVelocity();
        JointPosition joint_position = controller_->getPosition();
        JointVelocity joint_state_velocity = youbot_->arm()->getJointVelocity();
        JointPosition joint_state_position = youbot_->arm()->getJointPosition();

        // get torques
        JointAcceleration joint_acceleration = (joint_velocity - last_velocity_) / dt;
        JointVector efforts = youbot_dynamics_->getEffort(joint_position, joint_velocity, joint_acceleration);
        JointVector torques = torque_controller_->getTorques(joint_position, joint_velocity,
                                                 joint_state_position, joint_state_velocity, efforts);

        last_velocity_ = joint_velocity;

        youbot_->arm()->setJointTorques(torques);
    }


    if(!controller_->isBusy())
    {
        // === SET END POSE ===
        JointPosition end_pose = end_pose_;
        end_pose.addOffset();
        youbot_->arm()->setJointPositions(end_pose);

        if(goalReached())
        {
            ROS_INFO("MMP: Goal Pose reached.");

            status_ = STATUS_IDLE;
            endMovement();
        }
    }

    // publish position error
    //    JointPosition err = controller_->getPositionError();
    //    luh_youbot_msgs::JointVector err_msg;
    //    err_msg.q1 = err.q1();
    //    err_msg.q2 = err.q2();
    //    err_msg.q3 = err.q3();
    //    err_msg.q4 = err.q4();
    //    err_msg.q5 = err.q5();
    //    err_msg.header.stamp = ros::Time::now();
    //    position_error_publisher_.publish(err_msg);
}

//############### ACTIVATE #############################################################################################
void ModuleMotionPlanner::activate()
{
    deactivated = false;
}

//############### DEACTIVATE ###########################################################################################
void ModuleMotionPlanner::deactivate()
{
    deactivated = true;
}

//############### EMERGENCY STOP #######################################################################################
void ModuleMotionPlanner::emergencyStop()
{

    if(joint_path_server_->isActive())
    {
        joint_path_server_->setAborted();
    }
    if(cartesian_path_server_->isActive())
    {
        cartesian_path_server_->setAborted();
    }
    if(cylindric_path_server_->isActive())
    {
        cylindric_path_server_->setAborted();
    }
    if(named_path_server_->isActive())
    {
        named_path_server_->setAborted();
    }
    if(joint_pose_server_->isActive())
    {
        joint_pose_server_->setAborted();
    }
    if(cartesian_pose_server_->isActive())
    {
        cartesian_pose_server_->setAborted();
    }
    if(cylindric_pose_server_->isActive())
    {
        cylindric_pose_server_->setAborted();
    }
    if(named_pose_server_->isActive())
    {
        named_pose_server_->setAborted();
    }

    controller_->stop();

    status_ = STATUS_IDLE;
    arm_is_busy_ = false;

}

//############### LOAD #################################################################################################
bool ModuleMotionPlanner::load(std::string posefile, std::string neighborfile)
{
    // === GET FULL FILENAMES ===
    {
        if(posefile[0] != '/')
        {
            std::string path = ros::package::getPath("luh_youbot_controller");
            path.append("/");
            path.append(posefile);
            posefile = path;
        }

        if(neighborfile[0] != '/')
        {
            std::string path = ros::package::getPath("luh_youbot_controller");
            path.append("/");
            path.append(neighborfile);
            neighborfile = path;
        }
    }

    ROS_INFO("MMP: Loading file '%s'...", posefile.c_str());

    // === READ POSES YAML ===
    {
        std::vector<YAML::Node> nodes = YAML::LoadAllFromFile(posefile);

        //        while(parser.GetNextDocument(doc))
        for(uint i=0; i<nodes.size();i++)
        {
            Pose pose;

            pose.name = nodes[i].begin()->first.as<std::string>();
            //            const YAML::Node &joint_poses = nodes[i][pose.name];
            YAML::Node joint_poses = nodes[i].begin()->second;

            for(YAML::const_iterator it=joint_poses.begin();it!=joint_poses.end();++it)
            {
                YAML::const_iterator map_it = it->begin();
                std::string joint_name = map_it->first.as<std::string>();
                double joint_value = map_it->second.as<double>();
                joint_value *= M_PI/180.0;

                if(!joint_name.compare("arm_joint_1"))
                    pose.joint_position.setQ1(joint_value);
                else if(!joint_name.compare("arm_joint_2"))
                    pose.joint_position.setQ2(joint_value);
                else if(!joint_name.compare("arm_joint_3"))
                    pose.joint_position.setQ3(joint_value);
                else if(!joint_name.compare("arm_joint_4"))
                    pose.joint_position.setQ4(joint_value);
                else if(!joint_name.compare("arm_joint_5"))
                    pose.joint_position.setQ5(joint_value);
            }

            poses_.push_back(pose);
        }

        ROS_INFO("MMP: %d poses loaded.", (int)poses_.size());
    }

    // === READ NEIGHBORS YAML ===
    {
        // create map for access by name
        for(uint i=0; i<poses_.size(); i++)
        {
            pose_map_[poses_[i].name] = &poses_[i];
        }

        ROS_INFO("MMP: Loading file '%s'...", neighborfile.c_str());
        YAML::Node doc = YAML::LoadFile(neighborfile);

        int count=0;
        for(YAML::const_iterator it=doc.begin();it!=doc.end();++it)
        {
            YAML::const_iterator map_it = it->begin();
            std::string first = map_it->first.as<std::string>();
            std::string second = map_it->second.as<std::string>();

            bool valid = true;
            if(pose_map_.find(first) == pose_map_.end())
            {
                ROS_WARN("MMP: Unknown pose '%s' in relation '%s: %s'",first.c_str(), first.c_str(), second.c_str());
                valid = false;
            }
            if(pose_map_.find(second) == pose_map_.end())
            {
                ROS_WARN("MMP: Unknown pose '%s' in relation '%s: %s'",second.c_str(), first.c_str(), second.c_str());
                valid = false;
            }
            if(valid)
            {
                count++;
                pose_map_[first]->neighbors.push_back(pose_map_[second]);
                pose_map_[second]->neighbors.push_back(pose_map_[first]);
            }
        }

        ROS_INFO("MMP: %d neighborhood relations loaded.", count);

        //        // for debugging: print all neighborhood relations
        //        for(uint i=0; i<poses_.size(); i++)
        //        {
        //            std::cout << poses_[i].name << ": " << std::endl;
        //            for(uint j=0; j<poses_[i].neighbors.size(); j++)
        //            {
        //                std::cout << "    - " << poses_[i].neighbors[j]->name << std::endl;
        //            }
        //        }

        for(uint i=0; i<poses_.size(); i++)
        {
            if(poses_[i].neighbors.empty())
                ROS_WARN("Pose %s has no neighbors defined.", poses_[i].name.c_str());
        }
    }

    // === CALCULATE COSTS ===
    {
        for(uint i=0; i<poses_.size(); i++)
        {
            for(uint j=0; j<poses_[i].neighbors.size(); j++)
            {
                double cost = costFunction(poses_[i], *poses_[i].neighbors[j]);
                poses_[i].costs.push_back(cost);
            }
        }
    }

    return true;
}

//############### START MOVEMENT #######################################################################################
void ModuleMotionPlanner::startMovement(const Path &path)
{
    if(path.empty())
    {
        ROS_ERROR("MMP: Pose is unreachable from current pose.");
        (this->*endAction)(false);
        return;
    }

    // === CALCULATE TRAJECTORY ===
    JointPosition joint_position = youbot_->arm()->getJointPosition();
    controller_->newTrajectory(joint_position);

    ROS_INFO("MMP: Velocity factor is %f.", velocity_factor_);

    for(uint i=1; i<path.size(); i++)
    {
        controller_->setTargetJointVelocity(velocity_factor_);
        controller_->setPathPoint(path[i]);
    }

    ROS_INFO("MMP: Calculating path...");
    controller_->offlineMotionPlanning();

    status_ = STATUS_ACTIVE;

    ROS_INFO("MMP: Arm is moving...");
    controller_->moveToPose();
    arm_is_busy_ = true;

    last_update_time_ = ros::Time::now();
    last_velocity_ = JointVelocity();
}

//############### END MOVEMENT #########################################################################################
void ModuleMotionPlanner::endMovement()
{
    arm_is_busy_ = false;

    // === CALL END ACTION CALLBACK ===
    (this->*endAction)(true);

}

//############### GET TRAJECTORY #######################################################################################
Path ModuleMotionPlanner::getPath(JointPosition start, JointPosition end)
{
    Pose* start_pose = findClosestPose(start);
    Pose* end_pose = findClosestPose(end);

    PoseSet open;
    start_pose->status = OPEN;
    start_pose->cost = 0;
    start_pose->is_start = true;
    start_pose->estimate = costFunction(*start_pose, *end_pose);
    open.push_back(start_pose);

    end_pose->is_goal = true;

    // A*
    Pose* current_pose;
    while(true)
    {
        // === FIND POSE WITH MIN COST IN OPEN ===
        current_pose = open.getMinCostPose();

        // === OPEN EMPTY? ===
        if(current_pose == NULL)
            break;

        // === GOAL REACHED ? ===
        if(current_pose->is_goal)
            break;

        // === UPDATE NEIGHBORS ===
        updateNeighborCosts(current_pose, end);

        // === ADD NEW NEIGHBORS TO OPEN ===
        addNeighbors(open, current_pose);

        // === MARK CURRENT POSE AS CLOSED ===
        current_pose->status = CLOSED;
    }

    // === EXTRACT TRAJECTORY ===
    Path path;

    ROS_INFO("MMP: Planned path (reverse order):");
    if(current_pose != NULL) // NULL means no path found -> return empty path
    {
        while(true)
        {
            ROS_INFO("       - %s", current_pose->name.c_str());

            path.push_front(current_pose->joint_position);

            if(current_pose->is_start)
                break;

            current_pose = current_pose->precursor;
        }

        path.back() = end;

        if(path.size() < 2)
            path.push_front(start);
        else
            path.front() = start;
    }

    resetPoses();

    return path;
}

//############### FIND CLOSEST POSE ####################################################################################
Pose* ModuleMotionPlanner::findClosestPose(JointPosition pose)
{
    double distance = 1e6;
    Pose* closest = NULL;
    for(uint i=0; i<poses_.size(); i++)
    {
        JointPosition pos1 = pose;
        Pose pos2 = poses_[i];

        // ignore q5
        pos1.setQ5(0);
        pos2.joint_position.setQ5(0);

        double d = Pose::weighted_euclidean_distance(pos1, pos2, joint_weights_);
        if(d < distance)
        {
            distance = d;
            closest = &poses_[i];
        }
    }

    return closest;
}

//############### UPDATE NEIGHBOR COSTS ################################################################################
void ModuleMotionPlanner::updateNeighborCosts(Pose *current_pose, JointPosition end_position)
{
    for(uint i=0; i<current_pose->neighbors.size(); i++)
    {
        Pose* n = current_pose->neighbors[i];
        if(n->status == UNKNOWN)
        {
            n->cost = current_pose->cost + current_pose->costs[i];
            n->precursor = current_pose;
            n->estimate = costFunction(end_position, *n);
        }
        else if(n->status == OPEN)
        {
            double new_cost = current_pose->cost + current_pose->costs[i];

            if(new_cost < n->cost)
            {
                n->cost = new_cost;
                n->precursor = current_pose;
            }
        }
    }
}

//############### COST FUNCTION ########################################################################################
double ModuleMotionPlanner::costFunction(const Pose &pos1, const Pose &pos2)
{
    return costFunction(pos1.joint_position, pos2);
}

//############### COST FUNCTION ########################################################################################
double ModuleMotionPlanner::costFunction(const JointPosition &position, const Pose &pose)
{
    return Pose::duration(position, pose, joint_velocities_)
            + euclidean_influence_factor_ * Pose::weighted_euclidean_distance(position, pose, joint_weights_)
            + constant_influence_;
}

//############### ADD NEIGHBORS ########################################################################################
void ModuleMotionPlanner::addNeighbors(PoseSet &open, Pose *current_pose)
{
    for(uint i=0; i<current_pose->neighbors.size(); i++)
    {
        Pose* n = current_pose->neighbors[i];
        if(n->status == UNKNOWN)
        {
            n->status = OPEN;
            open.push_back(n);
        }

    }
}

//############### RESET POSES ##########################################################################################
void ModuleMotionPlanner::resetPoses()
{
    for(uint i=0; i<poses_.size(); i++)
    {
        poses_[i].status = UNKNOWN;
        poses_[i].is_goal = false;
        poses_[i].is_start = false;
    }
}

//############### IS READY #############################################################################################
bool ModuleMotionPlanner::isReady()
{
    ROS_INFO("==== Module Motion Planner ====");

    if(deactivated)
        return false;

    JointPosition joint_position = youbot_->arm()->getJointPosition();
    if(joint_position.empty())
    {
        ROS_ERROR("MMP: No jointstate message received yet. Cannot plan path.");
        return false;
    }

    if(arm_is_busy_)
    {
        ROS_ERROR("MMP: Arm is busy. Can't accept new goals.");
        return false;
    }

    return true;
}

//############### GOAL REACHED #########################################################################################
bool ModuleMotionPlanner::goalReached()
{
    luh_youbot_kinematics::JointPosition end_pose = end_pose_;
    end_pose.addOffset();
    luh_youbot_kinematics::JointPosition goal_pose = youbot_->arm()->getJointPosition();
    luh_youbot_kinematics::JointVelocity velocity = youbot_->arm()->getJointVelocity();
    luh_youbot_kinematics::JointPosition pose_diff = (goal_pose - end_pose).abs();

    //std::cout << "finepos_pos_tolerance = " << finepos_pos_tolerance_ * 180/M_PI << std::endl;
    //pose_diff.printValues("pose_diff");


    for(uint i=0; i<pose_diff.size(); i++)
    {
        if(pose_diff[i] > finepos_pos_tolerance_
                || fabs(velocity[i]) > finepos_vel_tolerance_)
        {
            return false;
        }
    }

    return true;
}
