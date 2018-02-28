/*
 * RosCommunication.cpp
 *
 *  Created on: 17 jan. 2018
 *      Author: Thomas
 */

#include "RosCommunication.hpp"


RosCommunication::RosCommunication()  :   pickupLocationPublisher(publisherNode.advertise<robotapplication::PickAndPlace>("pickupTarget", 1000))
{
}

 void RosCommunication::sendPickupLocation(robotapplication::PickAndPlace& message)
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
