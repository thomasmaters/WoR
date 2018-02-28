/*
 * Servo.hpp
 *
 *  Created on: 19 feb. 2017
 *      Author: Thomas
 */

#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <cstdint>
#include <iostream>
#include <string>

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
    Servo(uint8_t channel, std::string name, int16_t minRotation,
          int16_t maxRotation, uint16_t minPulseWidth, uint16_t maxPulseWidth,
          int16_t minSafeRotation, int16_t maxSafeRotation, double maxSpeed, bool inverted = false)
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
     * Checks if this servo can move to a specific rotation.
     * @param degrees	Rotation in degrees to check.
     * @return			True if possible.
     **/
    bool canMoveToDegrees(int16_t degrees) const
    {
        return degrees >= minRotation && degrees <= maxRotation &&
               degrees >= minSafeRotation && degrees <= maxSafeRotation;
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
    int16_t getDegreesFromPulseWidth(uint16_t pulseWidth) const
    {
        if (!canMoveToPulseWidth(pulseWidth)) {
            return 0;
        }

        if(inverted)
        {
        	return (((double)minRotation - maxRotation) /
					(maxPulseWidth - minPulseWidth)) *
					   (pulseWidth - minPulseWidth) +
				   maxRotation;
        }
        else
        {
			return (((double)maxRotation - minRotation) /
					(maxPulseWidth - minPulseWidth)) *
					   (pulseWidth - minPulseWidth) +
				   minRotation;
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
        if (!canMoveToDegrees(degrees)) {
            return 0;
        }

        if(inverted)
        {
            return (((double)minPulseWidth - maxPulseWidth) /
                    (maxRotation - minRotation)) *
                       (degrees - minRotation) +
					   maxPulseWidth;
        }
        else
        {
            return (((double)maxPulseWidth - minPulseWidth) /
                    (maxRotation - minRotation)) *
                       (degrees - minRotation) +
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

  private:
    uint8_t channel;
    std::string name;

    int16_t minRotation;
    int16_t maxRotation;

    uint16_t minPulseWidth;
    uint16_t maxPulseWidth;

    int16_t minSafeRotation;
    int16_t maxSafeRotation;

    // per 60 degrees
    double maxSpeed;

    bool inverted;
};

#endif /* SERVO_HPP_ */
