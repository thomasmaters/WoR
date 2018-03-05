/*
 * ssc32u_parser_to_jointstate.h
 *
 *  Created on: Mar 2, 2018
 *      Author: Thomas Maters
 */

#ifndef WOR_ROBOTSIMULATION_SRC_SSC32U_PARSER_TO_JOINTSTATE_H_
#define WOR_ROBOTSIMULATION_SRC_SSC32U_PARSER_TO_JOINTSTATE_H_

#include "../../shared/src/Servo.hpp"
#include "sensor_msgs/JointState.h"
#include "ssc32u_parser_interface.h"

#include <mutex>
#include <thread>

#include "ros/console.h"
#include "ros/master.h"

#define FPS 2

class JointStateConverter : public Ssc32uParserInterface
{
  public:
    JointStateConverter();

    void constructJointStateMessage();

    void ssc32uCommandReceived(const robotsimulation::ssc32u_command msg);

    virtual ~JointStateConverter();

  private:
    void moveToDesitnation(double movement_time,
                           const std::map<int16_t, std::pair<int16_t, int16_t>>& desired_positions);

    void parseSsc32uCommand(const robotsimulation::ssc32u_command& msg);

    bool isValidSsc32uCommand(double movement_time,
                              const std::map<int16_t, std::pair<int16_t, int16_t>>& desired_positions) const;

  private:
    sensor_msgs::JointState joint_state_msg_;

    std::vector<Servo> connected_servos_;
    std::map<uint8_t, double> current_positions_;

    std::thread moving_thread_;
    std::thread concurrent_joint_state_publish_thread_;  /// Concurrently publishes the current positions of the servos
                                                         /// on a topic.
    std::mutex publish_mutex;

    bool cancel_movement_;
    bool at_destination_;
    bool stopping_;
};

#endif /* WOR_ROBOTSIMULATION_SRC_SSC32U_PARSER_TO_JOINTSTATE_H_ */
