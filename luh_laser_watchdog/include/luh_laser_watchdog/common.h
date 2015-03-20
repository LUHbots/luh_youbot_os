#ifndef LUH_LASER_WATCHDOG_COMMON_H
#define LUH_LASER_WATCHDOG_COMMON_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "luh_laser_watchdog/Distances.h"

namespace luh_laser_watchdog
{

struct Point2D
{
    double x, y;
};

typedef std::vector<Point2D> Footprint;

}
#endif // LUH_LASER_WATCHDOG_COMMON_H
