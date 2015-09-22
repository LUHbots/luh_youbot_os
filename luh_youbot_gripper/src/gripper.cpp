#include "luh_youbot_gripper/gripper.h"

#define READ_GRIPPER_STATE 0
#define SET_POSITION 2
#define SET_EFFORT 4
#define SET_RELATIVE_POSITION 8

namespace luh_youbot_gripper
{

//########## CONSTRUCTOR ###############################################################################################
Gripper::Gripper():
    port_(io_),
    port_name_("/dev/arduino"),
    baud_rate_(57600)
{

}

//########## DESTRUCTOR ################################################################################################
Gripper::~Gripper()
{
    port_.close();
}

//########## INITIALISE ##############################################################################################
void Gripper::initialise()
{
    // === GET PARAMETERS ===
    bool all_params_loaded =
            ros::param::get("luh_gripper/serial_port", port_name_) &&
            ros::param::get("luh_gripper/baudrate", baud_rate_);

    if(!all_params_loaded)
        ROS_WARN("Not all parameters could be loaded. Default values will be used.");

    // === INITIALISE SERIAL PORT ===

    try
    {
        port_.open(port_name_.c_str());
        port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        ROS_INFO("Connected to %s at baud rate %d.", port_name_.c_str(), baud_rate_);
    }
    catch(boost::system::system_error &e)
    {
        ROS_ERROR("Could not connect to %s.", port_name_.c_str());
        ros::shutdown();
    }
}

//########## SET POSITION ##############################################################################################
void Gripper::setPosition(float left, float right)
{
    short angle_left = (int)left; // TODO: gripper kinematics
    short angle_right = (int)right; // TODO: gripper kinematics

    write(left, right, SET_POSITION);
}

//########## SET MAX EFFORT ############################################################################################
void Gripper::setMaxEffort(float left, float right)
{
    // TODO: range check
    write(left, right, SET_EFFORT);

}

//########## SET RELATIVE POSITION #####################################################################################
void Gripper::setRelativePosition(float left, float right)
{
    write(left, right, SET_RELATIVE_POSITION);
}

//########## GET STATE #################################################################################################
GripperState Gripper::getState()
{
    GripperState state;
    short* values = new short[8];//changed to 8 to measure the Vcc
    try
    {
        char flag = READ_GRIPPER_STATE;
        boost::asio::write(port_, boost::asio::buffer(&flag, 1));
        int b = boost::asio::read(port_, boost::asio::buffer(values, 16));
    }
    catch(boost::system::system_error &e)
    {
        ROS_WARN("%s", e.what());
    }

    state.position.first = values[0];
    state.position.second = values[1];
    state.velocity.first = values[2];
    state.velocity.second = values[3];
    state.effort.first = values[4];
    state.effort.second = values[5];
    state.goal_reached = values[6] > 0;
    state.usbVcc= values[7]; //usbVcc measurement

    delete[] values;

    return state;
}

//########## WRITE #####################################################################################################
void Gripper::write(short left, short right, char flag)
{
    short* values = new short[2];
    values[0] = left;
    values[1] = right;
    try
    {
        boost::asio::write(port_, boost::asio::buffer(&flag, 1));
        boost::asio::write(port_, boost::asio::buffer(values, 4));
    }
    catch(boost::system::system_error &e)
    {
        ROS_WARN("%s", e.what());
    }

    delete[] values;
}

}// namespace luh_youbot_gripper
