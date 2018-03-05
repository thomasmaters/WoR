/*
 * File:   ssc32u_parser_node.cpp
 * Author: Thomas Maters
 *
 * Created on March 21, 2017, 3:05 PM
 */

#include "ros/ros.h"

#include "ssc32u_parser_to_jointstate.h"

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "convert_node");

        ros::NodeHandle node_handle;
        if (!node_handle.hasParam("servos"))
        {
            ROS_ERROR("Failed to load parameter 'servos' from the param server.");
            return 1;
        }
        JointStateConverter parser;

        ros::spin();
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("ssc32u_parser_node crashed because of: " << e.what());
    }

    return 0;
}
