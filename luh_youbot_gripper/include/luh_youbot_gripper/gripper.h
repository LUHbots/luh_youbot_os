#ifndef LUH_ARDUINO_GRIPPER_H
#define LUH_ARDUINO_GRIPPER_H

#include <ros/ros.h>
#include <boost/asio.hpp>
#include "actionlib/server/simple_action_server.h"
#include "luh_youbot_msgs/GripObjectAction.h"
#include <luh_youbot_msgs/SetGripperAction.h>
#include <std_msgs/Float32.h>

namespace luh_youbot_gripper
{
struct GripperState
{
    std::pair<short, short> position;
    std::pair<short, short> velocity;
    std::pair<short, short> effort;
    bool goal_reached;
    short usbVcc; // Voltage Measurement on the USB
};

class Gripper
{
public:
    Gripper();
    ~Gripper();

    void initialise();
    void setPosition(float left, float right);
    void setMaxEffort(float left, float right);
    void setRelativePosition(float left, float right);
    GripperState getState();

protected:

    boost::asio::io_service io_;
    boost::asio::serial_port port_;
    std::string port_name_;
    int baud_rate_;

    void write(short left, short right, char flag);


};

} // namespace luh_youbot_gripper

#endif // LUH_ARDUINO_GRIPPER_H
