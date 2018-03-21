/*
 * virtualcupinterface.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Thomas Maters
 */

#ifndef WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_INTERFACE_H_
#define WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_INTERFACE_H_

#include <tf/transform_listener.h>
#include "robotsimulation/cup_data.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

/**
 * Virtual cups interface.
 * @author Thomas Maters
 */
class VirtualCupInterface
{
  public:
    VirtualCupInterface(const std::string& cup_name);

    /**
     * Sends cup data on a topic.
     * @param msg To send.
     * @author Thomas Maters
     */
    void sendCupData(const robotsimulation::cup_data& msg) const;

    /**
     * Sends marker data on a topic.
     * @param msg To send.
     * @author Thomas Maters
     */
    void sendMarkerData(const visualization_msgs::Marker& msg) const;

    /**
     * Uses tf listener to get a stamped transform between frames.
     * @param frame
     * @param frame_2
     * @return Stamped transfrom between frames.
     * @author Thomas Maters
     */
    tf::StampedTransform getTransform(const std::string& frame, const std::string& frame_2) const;

    virtual ~VirtualCupInterface();

  private:
    ros::NodeHandle publisher_node_;
    ros::Publisher cup_data_publisher_;
    ros::Publisher marker_data_publisher_;
    tf::TransformListener listener_;
};

#endif /* WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_INTERFACE_H_ */
