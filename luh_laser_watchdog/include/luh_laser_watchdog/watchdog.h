#ifndef LUH_LASER_WATCHDOG_H
#define LUH_LASER_WATCHDOG_H

#include "luh_laser_watchdog/collision_watchdog.h"
#include "luh_laser_watchdog/common.h"
#include "luh_laser_watchdog/laser.h"

namespace luh_laser_watchdog
{

class LaserWatchdog
{
public:
    LaserWatchdog(ros::NodeHandle &node);
    ~LaserWatchdog();

protected:

    ros::NodeHandle* node_;

    std::vector<Laser> lasers_;
    ros::Publisher distance_publisher_;    

    ros::Timer timer_;

    double publish_frequency_;    

    CollisionWatchdog* collision_watchdog_;

    void timerCallback(const ros::TimerEvent& evt);       
};

}

#endif // LUH_LASER_WATCHDOG_H
