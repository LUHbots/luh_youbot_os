#ifndef LUH_LASER_WATCHDOG_COLLISION_WATCHDOG_H
#define LUH_LASER_WATCHDOG_COLLISION_WATCHDOG_H

#include "luh_laser_watchdog/footprint_reader.h"
#include "luh_laser_watchdog/common.h"
#include <geometry_msgs/Twist.h>
#include "luh_laser_watchdog/laser.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

namespace luh_laser_watchdog
{

class CollisionWatchdog
{
public:
    CollisionWatchdog(ros::NodeHandle &node, std::vector<Laser> &lasers);

protected:

    ros::Subscriber cmd_vel_subscriber_;
    ros::Publisher cmd_vel_publisher_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_publisher_;


    Footprint footprint_;
    std::vector<Laser>& lasers_;

    // parameters
    double sample_step_;
    double progression_factor_;
    bool show_visualisation_;
    double critical_time_;
    int safety_margin_;

    cv::Mat visualisation_;
    double pixels_per_meter_;

    void velocityCallback(const geometry_msgs::Twist::ConstPtr &vel);

    Footprint predictFootprint(const geometry_msgs::Twist::ConstPtr &vel, double seconds);
    bool hasCollision(Footprint &footprint);
    bool pointInPolygon(Point2D point, Footprint& footprint);

    double getTimeToCollision(const geometry_msgs::Twist::ConstPtr &vel);
    void drawScanPoints();
    void drawFootprint(Footprint &footprint, cv::Scalar color, int thickness = 1);
    void drawMessage(double time);
    void publishImage();
};

}
#endif // LUH_LASER_WATCHDOG_COLLISION_WATCHDOG_H
