#ifndef LUH_LASER_WATCHDOG_LASER_H
#define LUH_LASER_WATCHDOG_LASER_H

#include "luh_laser_watchdog/common.h"

namespace luh_laser_watchdog
{

class Laser
{
public:
    Laser();
    void init(ros::NodeHandle &node, std::string topic);

    bool gotNewScan();
    Distances getDistances();
    std::vector<Point2D>& getScanPoints(){return scan_points_;}

protected:
    ros::NodeHandle* node_;

    std::string frame_id_;
    std::string topic_;
    tf::StampedTransform transform_;
    ros::Subscriber laser_subscriber_;
    std::string target_frame_;
    bool got_new_scan_;
    sensor_msgs::LaserScan last_scan_;
    std::vector<Point2D> scan_points_;
    double footprint_width_x_, footprint_width_y_;
    int num_points_for_mean_;

    // optimization
    std::vector<double> cos_phi_;
    std::vector<double> sin_phi_;

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

    double getMin(std::vector<double> &v);

};
}
#endif // LUH_LASER_WATCHDOG_LASER_H
