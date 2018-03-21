/*
 * ssc32u_parser_to_jointstate.h
 *
 *  Created on: Mar 2, 2018
 *      Author: Thomas Maters
 */

#ifndef WOR_ROBOTSIMULATION_SRC_SSC32U_PARSER_TO_JOINTSTATE_H_
#define WOR_ROBOTSIMULATION_SRC_SSC32U_PARSER_TO_JOINTSTATE_H_

#include "../../shared/src/SimulationServo.hpp"
#include "sensor_msgs/JointState.h"
#include "ssc32u_parser_interface.h"

#include <map>
#include <mutex>
#include <thread>

#include "ros/console.h"
#include "ros/master.h"

#define FPS 50

/**
 * Converts ssc32u command to jointstates.
 * @author Thomas Maters
 */
class JointStateConverter : public Ssc32uParserInterface
{
  public:
    /**
     * Constructor
     */
    JointStateConverter();

    /**
     * Constructs a joint state message from the current servo positions.
     * @author Thomas Maters
     */
    void constructJointStateMessage();

    /**
     * Callback that gets called when the parsers receives a ssc32u command.
     * @param msg Command message.
     * @author Thomas Maters
     */
    void ssc32uCommandReceived(const robotsimulation::ssc32u_command msg);

    virtual ~JointStateConverter();

  private:
    /**
     * Moves the a desired position in a amount of time.
     * @param movement_time Time to move.
     * @param desired_positions Positions to move to.
     * @author Thomas Maters
     */
    void moveToDesitnation(double movement_time,
                           const std::map<int16_t, std::pair<int16_t, int16_t>>& desired_positions);

    /**
     * Parses a ssc32u command where the servo's have to go to.
     * @param msg command to parse.
     * @author Thomas Maters
     */
    void parseSsc32uCommand(const robotsimulation::ssc32u_command& msg);

    /**
     * Validates if the command an positions are allowed to go to.
     * @param movement_time Time to move.
     * @param desired_positions Positions to go to.
     * @return True if valid.
     * @author Thomas Maters
     */
    bool isValidSsc32uCommand(double movement_time,
                              const std::map<int16_t, std::pair<int16_t, int16_t>>& desired_positions) const;

  private:
    sensor_msgs::JointState joint_state_msg_;

    std::map<uint8_t, SimulationServo> connected_servos_;
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
