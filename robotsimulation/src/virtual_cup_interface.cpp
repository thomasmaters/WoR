/*
 * virtualcupinterface.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Thomas Maters
 */

#include "virtual_cup_interface.h"

VirtualCupInterface::VirtualCupInterface(const std::string& cup_name)
{
    marker_data_publisher_ = publisher_node_.advertise<visualization_msgs::Marker>("cup_" + cup_name, 100);
    cup_data_publisher_ = publisher_node_.advertise<robotsimulation::cup_data>("cup_" + cup_name + "_data", 100);
}

void VirtualCupInterface::sendCupData(const robotsimulation::cup_data& msg) const
{
    cup_data_publisher_.publish(msg);
    ros::spinOnce();
}

void VirtualCupInterface::sendMarkerData(const visualization_msgs::Marker& msg) const
{
    marker_data_publisher_.publish(msg);
    ros::spinOnce();
}

tf::StampedTransform VirtualCupInterface::getTransform(const std::string& frame, const std::string& frame_2) const
{
    tf::StampedTransform stamped_transform;
    try
    {
        listener_.lookupTransform(frame, frame_2, ros::Time(0), stamped_transform);
    }
    catch (std::exception& e)
    {
        // std::cout << e.what() << std::endl;
    }

    return stamped_transform;
}

VirtualCupInterface::~VirtualCupInterface()
{
}
