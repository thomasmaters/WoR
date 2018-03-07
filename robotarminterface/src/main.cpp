#include "MotionControl.hpp"
#include "RosCommunication.hpp"
#include "ros/ros.h"

#include <iostream>
#include <thread>
#include "SerialControl.hpp"

// http://www.lynxmotion.com/p-1019-al5d-pltw-robotic-arm-kit.aspx
// https://www.servocity.com/hs-485hb-servo (base)
// https://www.servocity.com/hs-805bb-servo (shoulder)
// https://www.servocity.com/hs-755hb-servo (elbow)
// https://www.servocity.com/hs-645mg-servo (wrist)
// https://www.servocity.com/hs-85bb-servo (wrist rotate)
// https://www.servocity.com/hs-422-servo (gripper)

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robotarm_interface");
    // Can we start ros?
    if (MotionControl::getInstance().init())
    {
        ROS_INFO("Inited motion controller.");
        MotionControl::getInstance().setInterfaceState(interfaceStates::INIT);

        if (RosCommunication::getInstance().init())
        {
            ROS_INFO("Inited ros communication.");
            // Start the serial communication and the motion controller.
            if (SerialControl::getInstance().start("/dev/ttyUSB0", 9600))
            {
                ROS_INFO("Init completed correctly.");
                MotionControl::getInstance().setInterfaceState(interfaceStates::IDLE);
                MotionControl::getInstance().downloadStartupString("#0P1500;#1P1770;#2P1795;#3P796;#5P1527T3000");
                MotionControl::getInstance().moveToPosition(defaultPositions::PARK);
            }
            else
            {
                ROS_WARN("Failed to open serial port, commands will only be send on rostopic.");
                MotionControl::getInstance().setWaitForResponse(false);
                MotionControl::getInstance().setInterfaceState(interfaceStates::IDLE);
            }
        }
        else
        {
            ROS_ERROR("Coudln't start ros interface.");
            MotionControl::getInstance().setInterfaceState(interfaceStates::INIT_ERROR);
        }
    }
    else
    {
        ROS_ERROR("Coudln't initalize motion controller.");
    }

    ros::spin();
    return 0;
}
