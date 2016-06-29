#ifndef ARDUINO_GRIPPER_H
#define ARDUINO_GRIPPER_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <youbot/YouBotGripper.hpp>
#include <youbot/YouBotManipulator.hpp>

using namespace youbot;

class arduino_gripper
{
public:
    arduino_gripper();


//    struct GripperSetMessage{
//        unsigned int commandNumber;
//        unsigned int moduleAddress;
//        unsigned int typeNumber;
//        unsigned int motorNumber;
//        uint32 value;
//    };
//    struct GripperInputMessage{
//        unsigned int commandNumber;
//        unsigned int moduleAddress;
//        unsigned int replyAddress;
//        unsigned int status;
//        uint32 value;
//    };



    bool initAdruinoGripper(int slaveNumber,const std::string configFilePath = "../config/");
//    bool setMessage(GripperSetMessage OutputMessage);
//    void getStatus(GripperSetMessage OutputMessage, GripperInputMessage &InputMessage);
    void setPosition(float LengthBetweenGripperBarsInMeter);
    void getPosition(float& LengthBetweenGripperBarsInMeter);
    void setEffort(float EffortSetpoint);
    void getEffort(float& EffortSensed);
    void setVelocity(float MeterPerSecEachBar);
    void getVelocity(float& MeterPerSecEachBar);

private:
    YouBotGripper* m_pArduinoGripper;
};

#endif // ARDUINO_GRIPPER_H
