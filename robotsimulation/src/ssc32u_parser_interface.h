/*
 * RosCommunication.h
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#ifndef SRC_ROSCOMMUNICATION_H_
#define SRC_ROSCOMMUNICATION_H_

#include <map>
#include <vector>
#include "../../shared/src/Servo.hpp"
#include "robotsimulation/ssc32u_command.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class Ssc32uParserInterface
{
  public:
    Ssc32uParserInterface();
    virtual ~Ssc32uParserInterface();

  protected:
    virtual void ssc32uCommandReceived(const robotsimulation::ssc32u_command msg);

    void sendJointStateMessage(const sensor_msgs::JointState& msg) const;

    std::vector<Servo> getRosParamServoConfiguration() const;

  private:
    ros::NodeHandle subsriber_node_;
    ros::NodeHandle publisher_node_;
    ros::Subscriber target_position_subscriber_;
    ros::Publisher joint_state_publisher_;
};

#endif /* SRC_ROSCOMMUNICATION_H_ */
