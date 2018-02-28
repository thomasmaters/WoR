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
        : channel_(channel)
        , name_(name)
        , min_rotation_(minRotation)
        , max_rotation_(maxRotation)
        , min_pulse_width_(minPulseWidth)
        , max_pulse_width_(maxPulseWidth)
        , min_safe_rotation_(minSafeRotation)
        , max_safe_rotation_(maxSafeRotation)
        , max_speed_(maxSpeed)
  	  	, inverted_(inverted)
    {
    }

    /*Servo(Servo& other)
    {
    	if(this != &other)
    	{
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
    	}
    }*/

    /**
     * Checks if this servo can move to a specific rotation.
     * @param degrees	Rotation in degrees to check.
     * @return			True if possible.
     **/
    bool canMoveToDegrees(int16_t degrees) const
    {
        return degrees >= min_rotation_ && degrees <= max_rotation_ &&
               degrees >= min_safe_rotation_ && degrees <= max_safe_rotation_;
    }

    /**
     * Checks if the servo can move to a specific pulsewidth.
     * @param	pulseWidth	Pulse width in usec.
     * @return				True if possible.
     **/
    bool canMoveToPulseWidth(uint16_t pulseWidth) const
    {
        return pulseWidth >= min_pulse_width_ && pulseWidth <= max_pulse_width_;
    }

    /**
     * Converts a pulse width to degrees.
     * Returns 0 if the conversion is not possible.
     * @param	pulseWidth	Pulse width in usec.
     * @return	Returns the rotation in degrees corresponding that pulse width.
     **/
    int16_t getDegreesFromPulseWidth(uint16_t pulseWidth) const
    {
        /*if (!canMoveToPulseWidth(pulseWidth)) {
            return 0;
        }*/
    	if(pulseWidth < min_pulse_width_)
    	{
    		return inverted_ ? max_rotation_ : min_rotation_;
    	}
    	if(pulseWidth > max_pulse_width_)
    	{
    		return max_rotation_ ? max_rotation_ : min_rotation_;;
    	}

        if(inverted_)
        {
        	return (((double)min_rotation_ - max_rotation_) /
					(max_pulse_width_ - min_pulse_width_)) *
					   (pulseWidth - min_pulse_width_) +
				   max_rotation_;
        }
        else
        {
			return (((double)max_rotation_ - min_rotation_) /
					(max_pulse_width_ - min_pulse_width_)) *
					   (pulseWidth - min_pulse_width_) +
				   min_rotation_;
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

        if(inverted_)
        {
            return (((double)min_pulse_width_ - max_pulse_width_) /
                    (max_rotation_ - min_rotation_)) *
                       (degrees - min_rotation_) +
					   max_pulse_width_;
        }
        else
        {
            return (((double)max_pulse_width_ - min_pulse_width_) /
                    (max_rotation_ - min_rotation_)) *
                       (degrees - min_rotation_) +
                   min_pulse_width_;
        }

    }

    /**
     * Gets the servo's channel.
     * @return Returns the servo's channel.
     **/
    uint8_t getChannel() const
    {
        return channel_;
    }

    /**
     * Gets the servo's name.
     * @return Returns the servo's name.
     **/
    const std::string& getName() const
    {
        return name_;
    }

    virtual ~Servo()
    {
    }

  public:
    uint8_t channel_;
    std::string name_;

    int16_t min_rotation_;
    int16_t max_rotation_;

    uint16_t min_pulse_width_;
    uint16_t max_pulse_width_;

    int16_t min_safe_rotation_;
    int16_t max_safe_rotation_;

    // per 60 degrees
    double max_speed_;

    bool inverted_;
};

#endif /* SERVO_HPP_ */
