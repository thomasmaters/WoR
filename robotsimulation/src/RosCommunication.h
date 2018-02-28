/*
 * RosCommunication.h
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#ifndef SRC_ROSCOMMUNICATION_H_
#define SRC_ROSCOMMUNICATION_H_

#include "sensor_msgs/JointState.h"
#include "robotsimulation/ssc32u_command.h"
#include "ros/ros.h"
#include <vector>
#include <map>
#include "Servo.h"

class RosCommunication
{
  public:
    RosCommunication();

    void commandReceivedCallback(const robotsimulation::ssc32u_command msg);

    virtual ~RosCommunication();

  private:
    ros::NodeHandle subsriber_node_;
    ros::NodeHandle publisher_node_;
    ros::Subscriber target_position_subscriber_;
    ros::Publisher joint_state_publisher_;

    std::vector<Servo> connected_servos_;
    std::map<uint8_t, double> current_positions_;
};

#endif /* SRC_ROSCOMMUNICATION_H_ */
