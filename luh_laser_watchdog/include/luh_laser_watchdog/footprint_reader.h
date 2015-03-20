#ifndef LUH_LASER_WATCHDOG_FOOTPRINT_READER_H
#define LUH_LASER_WATCHDOG_FOOTPRINT_READER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "luh_laser_watchdog/common.h"

namespace luh_laser_watchdog
{

class FootprintReader
{
public:

    Footprint readFootprint(ros::NodeHandle &nh);

protected:
    std::vector<geometry_msgs::Point> footprint_;

    bool readFootprintFromString( const std::string& footprint_string );
    void readFootprintFromXMLRPC( XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                  const std::string& full_param_name );
    std::vector<std::vector<float> > parseVVF( const std::string& input, std::string& error_return );
    double getNumberFromXMLRPC( XmlRpc::XmlRpcValue& value, const std::string& full_param_name );

};
}

#endif // LUH_LASER_WATCHDOG_FOOTPRINT_READER_H
