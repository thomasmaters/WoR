/*
 * RosCommunication.hpp
 *
 *  Created on: 17 jan. 2018
 *      Author: Thomas
 */

#ifndef ROSCOMMUNICATION_HPP_
#define ROSCOMMUNICATION_HPP_

#include "robotapplication/pick_and_place.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

class RosCommunication
{
  public:
    RosCommunication();

    void sendPickupLocation(robotapplication::pick_and_place& message);

    void sendDebugMarker(visualization_msgs::Marker& message);

    virtual ~RosCommunication();

  private:
    ros::NodeHandle publisherNode;
    ros::Publisher pickupLocationPublisher;
    ros::Publisher debugMarkerPublisher;
};

#endif /* ROSCOMMUNICATION_HPP_ */
