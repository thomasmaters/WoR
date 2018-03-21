/*
 * RosCommunication.h
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#ifndef SRC_ROSCOMMUNICATION_H_
#define SRC_ROSCOMMUNICATION_H_

#include "../../shared/src/SimulationServo.hpp"
#include "robotsimulation/ssc32u_command.h"

#include <map>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

/**
 * Interface for the parser node.
 * @author Thomas Maters
 */
class Ssc32uParserInterface
{
  public:
    Ssc32uParserInterface();
    virtual ~Ssc32uParserInterface();

  protected:
    /**
     * Gets called when a ssc32u command is received. Override this function.
     * @param msg Command received.
     * @author Thomas Maters
     */
    virtual void ssc32uCommandReceived(const robotsimulation::ssc32u_command msg);

    /**
     * Sends joint state messages on a topic.
     * @param msg To send.
     * @author Thomas Maters
     */
    void sendJointStateMessage(const sensor_msgs::JointState& msg) const;

    /**
     * Reads the ros param servo for the servo configuration.
     * @return Servo configuration.
     * @author Thomas Maters
     */
    std::map<uint8_t, SimulationServo> getRosParamServoConfiguration() const;

  private:
    ros::NodeHandle subsriber_node_;
    ros::NodeHandle publisher_node_;
    ros::Subscriber target_position_subscriber_;
    ros::Publisher joint_state_publisher_;
};

#endif /* SRC_ROSCOMMUNICATION_H_ */
