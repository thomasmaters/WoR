#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"

#include <iostream>

#include "ApplicationManager.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_vision_node");
    try
    {
    	std::cout << "Starting manager" << std::endl;
        ApplicationManager manager = ApplicationManager();
        manager.start();
    }
    catch (std::exception& e)
    {
        std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
        return -1;
    }

    cv::destroyAllWindows();
    return 0;
}
