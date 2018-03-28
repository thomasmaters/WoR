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

class RosCommunication
{
  public:
    RosCommunication();

    void sendPickupLocation(robotapplication::pick_and_place& message);

    virtual ~RosCommunication();

  private:
    ros::NodeHandle publisherNode;
    ros::Publisher pickupLocationPublisher;
};

#endif /* ROSCOMMUNICATION_HPP_ */
