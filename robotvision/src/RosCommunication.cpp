/*
 * RosCommunication.cpp
 *
 *  Created on: 17 jan. 2018
 *      Author: Thomas
 */

#include "RosCommunication.hpp"

RosCommunication::RosCommunication()
  : pickupLocationPublisher(publisherNode.advertise<robotapplication::pick_and_place>("pickupTarget", 1000))
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

RosCommunication::~RosCommunication()
{
    // TODO Auto-generated destructor stub
}
