/*
 * RosCommunication.cpp
 *
 *  Created on: 17 jan. 2018
 *      Author: Thomas
 */

#include "RosCommunication.hpp"

RosCommunication::RosCommunication()
  : pickupLocationPublisher(publisherNode.advertise<robotapplication::pick_and_place>("pickupTarget", 1000))
  , debugMarkerPublisher(publisherNode.advertise<visualization_msgs::Marker>("visionDebugMarkers", 50))
{
}

void RosCommunication::sendPickupLocation(robotapplication::pick_and_place& message)
{
    if (ros::ok())
    {
        pickupLocationPublisher.publish(message);
        ros::spinOnce();
    }
}

void RosCommunication::sendDebugMarker(visualization_msgs::Marker& message)
{
    if (ros::ok())
    {
        debugMarkerPublisher.publish(message);
        ros::spinOnce();
    }
}

RosCommunication::~RosCommunication()
{
    // TODO Auto-generated destructor stub
}
