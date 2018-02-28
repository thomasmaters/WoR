#include "ros/ros.h"
#include "RosCommunication.hpp"
#include "MotionControl.hpp"

#include "SerialControl.hpp"
#include <iostream>
#include <thread>

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
//	Servo kaas = Servo(0, "base", -95, 95, 553, 2425, -90, 90, 0.22);
//	Servo kaaz = Servo(1, "shoulder", -99, 99, 556, 2420, -30, 90, 0.19,true);
//	Servo kaab = Servo(2, "elbow", 0, 202, 556, 2410, 0, 135, 0.28);
//	Servo kaac = Servo(3, "wrist", -93, 93, 553, 2520, -90, 90, 0.24);
//	Servo kaaz = Servo(4, "gripper", -93, 93, 553, 2520, -90, 90, 0.21);
//	Servo kaad = Servo(5, "wrist rotate", -91, 90, 553, 2300, -90, 90, 0.16);

	//Can we start ros?
	if(MotionControl::getInstance().init())
	{
		std::cout << "Inited motion controller." << std::endl;
		MotionControl::getInstance().setInterfaceState(interfaceStates::INIT);

		if(RosCommunication::getInstance().init())
		{
			std::cout << "Inited ros communication." << std::endl;
			//Start the serial communication and the motion controller.
			if (SerialControl::getInstance().start("/dev/ttyUSB0", 9600))
			{
				std::cout << "Init completed" << std::endl;
				MotionControl::getInstance().setInterfaceState(interfaceStates::IDLE);
				MotionControl::getInstance().downloadStartupString("#0P1500;#1P1770;#2P1795;#3P796;#5P1527T3000");
				MotionControl::getInstance().moveToPosition(defaultPositions::PARK);
			}
			else
			{
				std::cout << "Failed to init Serial communication." << std::endl;
				MotionControl::getInstance().setInterfaceState(interfaceStates::INIT_ERROR);
			}
		}
		else
		{
			std::cout << "Couldn't start ros interface." << std::endl;
			MotionControl::getInstance().setInterfaceState(interfaceStates::INIT_ERROR);
		}
	}
	else
	{
		std::cout << "Faileed to init motion controller." << std::endl;
	}

//	std::cout << MotionControl::getInstance().getServoCurrentPulseWidth(0) << std::endl;

	ros::spin();
	return 0;
}
