#include "luh_youbot_driver_api/arduino_gripper.h"

arduino_gripper::arduino_gripper()
{
    std::cout << "Hey, Hier ist der geile Adruino-Gripper, ich kann jetzt über internen Bus kommunizieren!" << std::endl;

}

bool arduino_gripper::initAdruinoGripper(int slaveNumber, const std::string configFilePath)
{
    m_pArduinoGripper=new YouBotGripper(slaveNumber, configFilePath);

    //Hier eine Handschake Message senden und Response prüffen ob == NO_ERROR


}

//bool arduino_gripper::setMessage(GripperSetMessage OutputMessage)
//{
//    YouBotSlaveMailboxMsg MyMessage;

//    MyMessage.stctOutput.commandNumber = OutputMessage.commandNumber;
//    MyMessage.stctOutput.moduleAddress = OutputMessage.moduleAddress;
//    MyMessage.stctOutput.typeNumber = OutputMessage.typeNumber;
//    MyMessage.stctOutput.motorNumber = OutputMessage.motorNumber;
//    MyMessage.stctOutput.value = OutputMessage.value;


//    // eigentlich ist es eine Get Funktion, aber abhängig davon was man in die Message reischreibt
//    // kann man auch die Positionen setzen

//    m_pArduinoGripper->getConfigurationParameter(MyMessage);

//    GripperInputMessage InputMessage;

//    InputMessage.commandNumber=MyMessage.stctInput.commandNumber;
//    InputMessage.moduleAddress=MyMessage.stctInput.moduleAddress;
//    InputMessage.replyAddress=MyMessage.stctInput.replyAddress;
//    InputMessage.status=MyMessage.stctInput.status;
//    InputMessage.value=MyMessage.stctInput.value;

//    if(InputMessage.status == NO_ERROR)
//    {
//        std::cout << "LUH-GRIPPER got the message" << std::endl;
//    } else
//    {
//        std::cout << "LUH-GRIPPER got the message, but there is something wrong. Code:  "<< InputMessage.status << std::endl;
//    }
//}

//void arduino_gripper::getStatus(GripperSetMessage OutputMessage, GripperInputMessage& InputMessage)
//{
//    YouBotSlaveMailboxMsg MyMessage;

//    MyMessage.stctOutput.commandNumber = OutputMessage.commandNumber;
//    MyMessage.stctOutput.moduleAddress = OutputMessage.moduleAddress;
//    MyMessage.stctOutput.typeNumber = OutputMessage.typeNumber;
//    MyMessage.stctOutput.motorNumber = OutputMessage.motorNumber;
//    MyMessage.stctOutput.value = OutputMessage.value;


//    // eigentlich ist es eine Get Funktion, aber abhängig davon was man in die Message reischreibt
//    // kann man auch die Positionen setzen

//   m_pArduinoGripper->getConfigurationParameter(MyMessage);

//   InputMessage.commandNumber=MyMessage.stctInput.commandNumber;
//   InputMessage.moduleAddress=MyMessage.stctInput.moduleAddress;
//   InputMessage.replyAddress=MyMessage.stctInput.replyAddress;
//   InputMessage.status=MyMessage.stctInput.status;
//   InputMessage.value=MyMessage.stctInput.value;
////   InputMessage.value[0]=value>> 24;
////   InputMessage.value[1]=value>> 16;
////   InputMessage.value[2]=value>> 8;
////   InputMessage.value[3]=value>> & 0xff;


//   if(InputMessage.status == NO_ERROR)
//   {
//       std::cout << "LUH-GRIPPER got the message and sent the data back" << std::endl;
//   } else
//   {
//       std::cout << "LUH-GRIPPER got the message, but there is something wrong. Code:  "<< InputMessage.status << std::endl;
//   }

//}

void arduino_gripper::setPosition(float LengthBetweenGripperBarsInMeter)
{
    //int MaxLength=70; // in cm
    //int MaxAngle=144; //in Degrees


    uint32 AngleOfServoInDegrees=LengthBetweenGripperBarsInMeter*1000*360/M_PI/2/27.85; //27.85=Diameter of the Servo-Wheel
    std::cout << "Angle set to " << AngleOfServoInDegrees << std::endl;
    if(AngleOfServoInDegrees>144 || AngleOfServoInDegrees<0){
        std::cout << "Angle to set is out of range: " << AngleOfServoInDegrees << std::endl;
    }

    YouBotSlaveMailboxMsg MyMessage;

    MyMessage.stctOutput.moduleAddress = GRIPPER;
    MyMessage.stctOutput.commandNumber = MVP; // Move to Position
    MyMessage.stctOutput.typeNumber = 0;  // absolute
    MyMessage.stctOutput.motorNumber = 1; // always 1
    MyMessage.stctOutput.value = AngleOfServoInDegrees; // Position in Angle of the Servo Motor

    m_pArduinoGripper->getConfigurationParameter(MyMessage);

    std::cout << "Message set and the respose value is: " << MyMessage.stctInput.value << std::endl;

    if(!MyMessage.stctInput.status==100)
    {
        std::cout << "Something went terrible wrong, the respose status is not 100, but i just dont care and am going further!" << std::endl;
    }

}

void arduino_gripper::getPosition(float& LengthBetweenGripperBarsInMeter)
{
    YouBotSlaveMailboxMsg MyMessage;

    MyMessage.stctOutput.moduleAddress = GRIPPER;
    MyMessage.stctOutput.commandNumber = GAP; // get axis parameter
    MyMessage.stctOutput.typeNumber = 1;  // actual position
    MyMessage.stctOutput.motorNumber = 1; // always 1
    MyMessage.stctOutput.value = 0; // dont care if GAP

    m_pArduinoGripper->getConfigurationParameter(MyMessage);

    if(!MyMessage.stctInput.status==100)
    {
        std::cout << "Something went terrible wrong, the respose status is not 100, but i just dont care and am going further!" << std::endl;
    } else {
        LengthBetweenGripperBarsInMeter=MyMessage.stctInput.value*M_PI*27.85/360*2/1000; //27.85=Diameter of the Servo-Wheel
    }
}

void arduino_gripper::setEffort(float EffortSetpoint)
{
    uint32 ValueToSet=EffortSetpoint;
    //Kraft setzen
    YouBotSlaveMailboxMsg MyMessage;

    MyMessage.stctOutput.moduleAddress = GRIPPER;
    MyMessage.stctOutput.commandNumber = SAP; // SET AXIS PARAMETER
    MyMessage.stctOutput.typeNumber = 155;  //Target Current
    MyMessage.stctOutput.motorNumber = 1; //
    MyMessage.stctOutput.value = ValueToSet;

    m_pArduinoGripper->getConfigurationParameter(MyMessage);

    std::cout << "response: " << MyMessage.stctInput.replyAddress << std::endl;
    std::cout << "response: " << MyMessage.stctInput.moduleAddress << std::endl;
    std::cout << "response: " << MyMessage.stctInput.status << std::endl;
    std::cout << "response: " << MyMessage.stctInput.commandNumber << std::endl;
    std::cout << "response: " << MyMessage.stctInput.value << std::endl;

    if(!MyMessage.stctInput.status==100)
    {
        std::cout << "Something went terrible wrong, the respose status is not 100, but i just dont care and am going further!" << std::endl;
    }
}

void arduino_gripper::getEffort(float& EffortSensed)
{
    YouBotSlaveMailboxMsg MyMessage;

     MyMessage.stctOutput.moduleAddress = GRIPPER;
     MyMessage.stctOutput.commandNumber = GAP; // get axis parameter
     MyMessage.stctOutput.typeNumber = 150;  // ACTUAL_CURRENT
     MyMessage.stctOutput.motorNumber = 1; // always 1
     MyMessage.stctOutput.value = 0; // dont care if GAP

     m_pArduinoGripper->getConfigurationParameter(MyMessage);

     if(!MyMessage.stctInput.status==100)
     {
         std::cout << "Something went terrible wrong, the respose status is not 100, but i just dont care and am going further!" << std::endl;
     } else {
         EffortSensed=MyMessage.stctInput.value;
     }

}

void arduino_gripper::setVelocity(float MeterPerSecEachBar)
{
    uint32 ValueToSet=MeterPerSecEachBar;
    //Kraft setzen
    YouBotSlaveMailboxMsg MyMessage;

    MyMessage.stctOutput.moduleAddress = GRIPPER;
    MyMessage.stctOutput.commandNumber = SAP; // SET AXIS PARAMETER
    MyMessage.stctOutput.typeNumber = 2;  //Target Speed
    MyMessage.stctOutput.motorNumber = 1; //
    MyMessage.stctOutput.value = ValueToSet;

    m_pArduinoGripper->getConfigurationParameter(MyMessage);

    std::cout << "response: " << MyMessage.stctInput.replyAddress << std::endl;
    std::cout << "response: " << MyMessage.stctInput.moduleAddress << std::endl;
    std::cout << "response: " << MyMessage.stctInput.status << std::endl;
    std::cout << "response: " << MyMessage.stctInput.commandNumber << std::endl;
    std::cout << "response: " << MyMessage.stctInput.value << std::endl;

    if(!MyMessage.stctInput.status==100)
    {
        std::cout << "Something went terrible wrong, the respose status is not 100, but i just dont care and am going further!" << std::endl;
    }

}

void arduino_gripper::getVelocity(float& MeterPerSecEachBar)
{
     YouBotSlaveMailboxMsg MyMessage;

     MyMessage.stctOutput.moduleAddress = GRIPPER;
     MyMessage.stctOutput.commandNumber = GAP; // get axis parameter
     MyMessage.stctOutput.typeNumber = 2;  // Target Speed
     MyMessage.stctOutput.motorNumber = 1; // always 1
     MyMessage.stctOutput.value = 0; // dont care if GAP

     m_pArduinoGripper->getConfigurationParameter(MyMessage);

     if(!MyMessage.stctInput.status==100)
     {
         std::cout << "Something went terrible wrong, the respose status is not 100, but i just dont care and am going further!" << std::endl;
     } else {
         MeterPerSecEachBar=MyMessage.stctInput.value;
     }

}
