#include <ros/ros.h>
//#include <urdf/model.h>

#include "virtual_cup.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cup_node", ros::init_options::AnonymousName);

    if (argc != 4)
    {
        ROS_ERROR_STREAM("Not enough parameters for launching the cup node. (Expected 3, got " << (argc - 1) << ")");
        return -1;
    }

    Cup cup(ros::this_node::getName(), std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]));
    cup.loop();
    return 0;
}
