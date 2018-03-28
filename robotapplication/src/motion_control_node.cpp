/**
 * @file   main.cpp
 * @brief  The main
 * @author Thomas Maters
 * @author Peter van Leeuwen
 */
#include <iostream>
#include "InverseKinematics.h"
#include "Matrix.hpp"
#include "motion_control_controller.h"
#include "ros/console.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "motion_control_node");
        MotionControl communication;
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
