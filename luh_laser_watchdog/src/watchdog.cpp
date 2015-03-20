#include "luh_laser_watchdog/watchdog.h"

using namespace luh_laser_watchdog;

//########## WATCHDOG CONSTRUCTOR ######################################################################################
LaserWatchdog::LaserWatchdog(ros::NodeHandle &node):
    node_(&node), collision_watchdog_(NULL)
{
    // === PARAMETERS ===
    std::vector<std::string> laser_topics;
    node_->param("laser_watchdog/laser_topics", laser_topics, std::vector<std::string>(1, "/scan"));
    node_->param("laser_watchdog/publish_frequency", publish_frequency_, 10.0);

    // === LASERS ===
    lasers_.resize(laser_topics.size());
    for(uint i=0; i<laser_topics.size(); i++)
    {
        lasers_[i].init(node, laser_topics[i]);
    }
    collision_watchdog_ = new CollisionWatchdog(node, lasers_);

    // === PUBLISHERS ===
    distance_publisher_ = node_->advertise<Distances>("laser_watchdog/distances", 10);

    // === TIMER ===
    timer_ = node_->createTimer(ros::Duration(1.0/publish_frequency_), &LaserWatchdog::timerCallback, this);
}

//########## WATCHDOG DESTRUCTOR #######################################################################################
LaserWatchdog::~LaserWatchdog()
{
    delete collision_watchdog_;
}

//########## CALLBACK: TIMER ###########################################################################################
void LaserWatchdog::timerCallback(const ros::TimerEvent& evt)
{
    // check if all scans are updated
    for(uint i=0; i<lasers_.size(); i++)
    {
        if(!lasers_[i].gotNewScan())
            return;
    }

    // merge distances from all lasers
    Distances dist;
    dist.front = 10.0;
    dist.back  = 10.0;
    dist.left  = 10.0;
    dist.right = 10.0;

    for(uint i=0; i<lasers_.size(); i++)
    {
        Distances dist_i = lasers_[i].getDistances();
        dist.front = std::min(dist.front, dist_i.front);
        dist.back  = std::min(dist.back, dist_i.back);
        dist.left  = std::min(dist.left, dist_i.left);
        dist.right = std::min(dist.right, dist_i.right);
    }

    distance_publisher_.publish(dist);
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_watchdog");

    ros::NodeHandle node_handle;
    LaserWatchdog watchdog(node_handle);


    ROS_INFO("Laser watchdog is spinning...");
    ros::spin();

    return 0;
}
