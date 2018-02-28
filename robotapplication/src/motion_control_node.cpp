/**
 * @file   main.cpp
 * @brief  The main
 * @author Thomas Maters
 * @author Peter van Leeuwen
 */
#include <iostream>
#include "InverseKinematics.h"
#include "Matrix.hpp"
#include "RosCommunication.h"
#include "ros/console.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "motion_control_node");
        RosCommunication communication;
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
