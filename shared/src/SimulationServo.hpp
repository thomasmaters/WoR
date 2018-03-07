/*
 * SimulationServo.hpp
 *
 *  Created on: 6 mar. 2018
 *      Author: Thomas Maters
 */

#ifndef SIMULATION_SERVO_HPP_
#define SIMULATION_SERVO_HPP_

#include "Servo.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <iostream>

class SimulationServo : public Servo
{
  public:
    enum JointTypes
    {
        revolute = 0,
        continuous = 1,
        prismatic = 2,
        fixed = 3,
        floating = 4,
        planar = 5
    };

    /**
     * Constructor
     * @param channel			Channel the servo is connected too.
     * @param name				Name of the servo.
     * @param minRotation		Minimum rotation in degrees the servo can rotate
     *to.
     * @param maxRotation		Maximum rotation in degrees the servo can rotate
     *to.
     * @param minPulseWidth		Minimum pulse width in useq the servo can rotate
     *to.
     * @param maxPulseWidth		Maximum pulse width in useq the servo can rotate
     *to.
     * @param minSafeRotation	Minimum rotation in degrees it is safe to rotate
     *the servo in.
     * @param maxSafeRotation	Maximum rotation in degrees it is safe to rotate
     *the servo in.
     * @param maxSpeed			Speed in useq per 60 degrees.
     * @param inverted			If the servo is reversed.
     * @param jointType			The type of joint of this simulation servo.
     * @param minJointValue		The minimal value of this simulation servo.
     * @param maxJointValue		The maximal value of this simulation servo.
     **/
    SimulationServo(uint8_t channel, std::string name, double_t minRotation, double_t maxRotation,
                    uint16_t minPulseWidth, uint16_t maxPulseWidth, double_t minSafeRotation, double_t maxSafeRotation,
                    double_t maxSpeed, bool inverted = false, JointTypes jointType = JointTypes::continuous,
                    double_t minJointValue = 0, double_t maxJointValue = 0)
      : Servo(channel, name, minRotation, maxRotation, minPulseWidth, maxPulseWidth, minSafeRotation, maxSafeRotation,
              maxSpeed, inverted)
      , jointType(jointType)
      , minJointValue(minJointValue)
      , maxJointValue(maxJointValue)
    {
    }

    /**
     * Copy constructor.
     */
    SimulationServo(const SimulationServo& other)
      : Servo(other), jointType(other.jointType), minJointValue(other.minJointValue), maxJointValue(other.maxJointValue)
    {
    }

    /**
     * Assignment operator.
     */
    SimulationServo& operator=(const SimulationServo& other)
    {
        if (&other == this)
        {
            return *this;
        }

        Servo::operator=(other);
        jointType = other.jointType;
        minJointValue = other.minJointValue;
        maxJointValue = other.maxJointValue;

        return *this;
    }

    double_t getJointStateFromPulseWidth(uint16_t pulseWidth) const
    {
        if (jointType != JointTypes::prismatic)
        {
            return Servo::getRadiansFromPulseWidth(pulseWidth);
        }
        return getPrismaticJointValueFromPulseWidth(pulseWidth);
    }

    double_t getPrismaticJointValueFromPulseWidth(uint16_t pulseWidth) const
    {
        if (!canMoveToPulseWidth(pulseWidth))
        {
            return 0;
        }

        if (inverted)
        {
            return (((minJointValue - maxJointValue) / (maxPulseWidth - minPulseWidth)) * (pulseWidth - minPulseWidth) +
                    maxJointValue);
        }
        else
        {
            return (((maxJointValue - minJointValue) / (maxPulseWidth - minPulseWidth)) * (pulseWidth - minPulseWidth) +
                    minJointValue);
        }
    }

  protected:
    JointTypes jointType;
    double_t minJointValue;
    double_t maxJointValue;
};

#endif /* SIMULATION_SERVO_HPP_ */
