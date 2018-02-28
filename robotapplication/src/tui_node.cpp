/**
 * @file   main.cpp
 * @brief  The main
 * @author Thomas Maters
 * @author Peter van Leeuwen
 */
#include <iostream>

#include <ros/ros.h>

#include "robotalgorithmfork/src/Reader.hpp"
#include "robotalgorithmfork/src/Detector.hpp"

const cv::String keys =
        "{help h ?     |      | print help             }"
        ;

// 0 0 45 2 45 4 45 2
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tui_node");
    
    Detector detector;

    cv::CommandLineParser parser(argc, (const char * const *) argv, keys);
    parser.about("RobotAlgorithm v1.0.0");
    
    if (parser.has("help")) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    if (!parser.check()) {
        parser.printErrors();
        return EXIT_SUCCESS;
    }
    
    if (!detector.openCam(1)) {
        return EXIT_FAILURE;
    }
    detector.interactiveMode();

    return EXIT_SUCCESS;
}
