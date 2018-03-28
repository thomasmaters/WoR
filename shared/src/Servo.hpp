/*
 * Servo.hpp
 *
 *  Created on: 19 feb. 2017
 *      Author: Thomas
 */

#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <cmath>
#include <cstdint>
#include <string>

#include <ros/console.h>

class Servo
{
  public:
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
     **/
    Servo(uint8_t channel, std::string name, double_t minRotation, double_t maxRotation, uint16_t minPulseWidth,
          uint16_t maxPulseWidth, double_t minSafeRotation, double_t maxSafeRotation, double_t maxSpeed,
          bool inverted = false)
      : channel(channel)
      , name(name)
      , minRotation(minRotation)
      , maxRotation(maxRotation)
      , minPulseWidth(minPulseWidth)
      , maxPulseWidth(maxPulseWidth)
      , minSafeRotation(minSafeRotation)
      , maxSafeRotation(maxSafeRotation)
      , maxSpeed(maxSpeed)
      , inverted(inverted)
    {
    }

    /**
     * Copy constructor
     */
    Servo(const Servo& other)
      : channel(other.channel)
      , name(other.name)
      , minRotation(other.minRotation)
      , maxRotation(other.maxRotation)
      , minPulseWidth(other.minPulseWidth)
      , maxPulseWidth(other.maxPulseWidth)
      , minSafeRotation(other.minSafeRotation)
      , maxSafeRotation(other.maxSafeRotation)
      , maxSpeed(other.maxSpeed)
      , inverted(other.inverted)
    {
    }

    /**
     * Assignment operator.
     */
    Servo& operator=(const Servo& other)
    {
        if (&other == this)
        {
            return *this;
        }

        channel = other.channel;
        name = other.name;
        minRotation = other.minRotation;
        maxRotation = other.maxRotation;
        minPulseWidth = other.minPulseWidth;
        maxPulseWidth = other.maxPulseWidth;
        minSafeRotation = other.minSafeRotation;
        maxSafeRotation = other.maxSafeRotation;
        maxSpeed = other.maxSpeed;
        inverted = other.inverted;

        return *this;
    }

    /**
     * Checks if this servo can move to a specific rotation.
     * @param degrees	Rotation in degrees to check.
     * @return			True if possible.
     **/
    bool canMoveToDegrees(int16_t degrees) const
    {
        return degrees >= minRotation && degrees <= maxRotation && degrees >= minSafeRotation &&
               degrees <= maxSafeRotation;
    }

    /**
     * Checks if the servo can move to a specific pulsewidth.
     * @param	pulseWidth	Pulse width in usec.
     * @return				True if possible.
     **/
    bool canMoveToPulseWidth(uint16_t pulseWidth) const
    {
        return pulseWidth >= minPulseWidth && pulseWidth <= maxPulseWidth;
    }

    /**
     * Converts a pulse width to degrees.
     * Returns 0 if the conversion is not possible.
     * @param	pulseWidth	Pulse width in usec.
     * @return	Returns the rotation in degrees corresponding that pulse width.
     **/
    virtual double_t getDegreesFromPulseWidth(uint16_t pulseWidth) const
    {
        if (!canMoveToPulseWidth(pulseWidth))
        {
            return 0;
        }

        if (inverted)
        {
            return ((minRotation - maxRotation) / (maxPulseWidth - minPulseWidth)) * (pulseWidth - minPulseWidth) +
                   maxRotation;
        }
        else
        {
            return ((maxRotation - minRotation) / (maxPulseWidth - minPulseWidth)) * (pulseWidth - minPulseWidth) +
                   minRotation;
        }
    }

    /**
     * Converts a pulse width to radians.
     * Returns 0 if the conversion is not possible.
     * @param pulseWidth Pulse width in usec.
     * @return Returns the rotation in radians corresponding with the given pulse width.
     */
    virtual double_t getRadiansFromPulseWidth(uint16_t pulseWidth) const
    {
        if (!canMoveToPulseWidth(pulseWidth))
        {
            return 0;
        }

        if (inverted)
        {
            return (((minRotation - maxRotation) / (maxPulseWidth - minPulseWidth)) * (pulseWidth - minPulseWidth) +
                    maxRotation) *
                   M_PI / 180;
        }
        else
        {
            return (((maxRotation - minRotation) / (maxPulseWidth - minPulseWidth)) * (pulseWidth - minPulseWidth) +
                    minRotation) *
                   M_PI / 180;
        }
    }

    /**
     * Converts a rotation in degrees to a pulse width.
     * Returns 0 if the conversion is not possible.
     * @param	degrees	Rotation in degrees to convert.
     * @return	Returns the pulse width in useq corresponding that rotation.
     **/
    uint16_t getPulseWidthFromDegrees(int16_t degrees) const
    {
        if (!canMoveToDegrees(degrees))
        {
            if (degrees < minRotation)
            {
                ROS_WARN("A lower then possible degrees has been given, returning lowest possible value.");
                return inverted ? maxPulseWidth : minPulseWidth;
            }
            if (degrees > maxRotation)
            {
                ROS_WARN("A higher then possible degrees has been given, returning highest possible value.");
                return inverted ? minPulseWidth : maxPulseWidth;
            }
        }

        if (inverted)
        {
            return (((double)minPulseWidth - maxPulseWidth) / (maxRotation - minRotation)) * (degrees - minRotation) +
                   maxPulseWidth;
        }
        else
        {
            return (((double)maxPulseWidth - minPulseWidth) / (maxRotation - minRotation)) * (degrees - minRotation) +
                   minPulseWidth;
        }
    }

    /**
     * Gets the servo's channel.
     * @return Returns the servo's channel.
     **/
    uint8_t getChannel() const
    {
        return channel;
    }

    /**
     * Gets the servo's name.
     * @return Returns the servo's name.
     **/
    const std::string& getName() const
    {
        return name;
    }

    virtual ~Servo()
    {
    }

    uint16_t getMaxPulseWidth() const
    {
        return maxPulseWidth;
    }

    uint16_t getMinPulseWidth() const
    {
        return minPulseWidth;
    }

  protected:
    uint8_t channel;
    std::string name;

    double_t minRotation;
    double_t maxRotation;

    uint16_t minPulseWidth;
    uint16_t maxPulseWidth;

    double_t minSafeRotation;
    double_t maxSafeRotation;

    // per 60 degrees
    double_t maxSpeed;

    bool inverted;
};

#endif /* SERVO_HPP_ */
