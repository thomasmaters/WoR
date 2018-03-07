/*
 * MovementCommand.hpp
 *
 *  Created on: 21 feb. 2017
 *      Author: Thomas
 */

#ifndef MOVEMENTCOMMAND_HPP_
#define MOVEMENTCOMMAND_HPP_

#include <cstdint>
#include <string>

class MovementCommand
{
  public:
    /**
     * Constructor.
     * @param channel		Channel the servo is connected to.
     * @param pulseWidth	Pulse width in useq to move the servo to.
     * @param speed			Speed in useq/sec to move the servo in.
     * @param time			Time in miliseconds to move the servo
     *in.
     **/
    MovementCommand(uint8_t channel, uint16_t pulseWidth, uint16_t speed, uint16_t time = 0)
        : channel(channel), pulseWidth(pulseWidth), speed(speed), time(time), command("")
    {
    }

    /**
     * Constructor.
     * @param command		Muliservo movement command.
     **/
    MovementCommand(std::string command) : channel(0), pulseWidth(0), speed(0), time(0), command(command)
    {
    }

    /**
     * Constructs the a message the controller understands given the class
     *variables.
     * @return Returns a movement command for the controller.
     **/
    std::string getCommandString()
    {
        // Do we have a movementgroup command?
        if (!command.empty())
        {
            return command;
        }

        // Do we have a time in our movement command?
        if (time == 0)
        {
            return "#" + std::to_string(channel) + "P" + std::to_string(pulseWidth) + "S" + std::to_string(speed) +
                   '\r';
        }

        return "#" + std::to_string(channel) + "P" + std::to_string(pulseWidth) + "T" + std::to_string(time) + '\r';
    }

    uint16_t getTime() const
    {
        return time;
    }

  private:
    uint8_t channel;
    uint16_t pulseWidth;
    uint16_t speed;
    uint16_t time;
    std::string command;
};

#endif /* MOVEMENTCOMMAND_HPP_ */
