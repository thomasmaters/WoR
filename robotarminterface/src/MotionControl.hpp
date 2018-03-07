/*
 * MotionControl.hpp
 *
 *  Created on: 18 feb. 2017
 *      Author: Thomas
 */

#ifndef MOTIONCONTROL_HPP_
#define MOTIONCONTROL_HPP_

#include "../../shared/src/Servo.hpp"
#include "MovementCommand.hpp"
#include "RosCommunication.hpp"
#include "SmartQueue.hpp"

#include <cstdint>
#include <string>
#include <vector>

#define MAX_STARTUP_STRING_LENGTH 100

enum defaultPositions
{
    PARK,
    READY,
    STRAIGHT_UP
};
enum interfaceStates
{
    IDLE,
    SENDING_COMMAND,
    MOVING_ARM,
    MOVING_FINISHED,
    INIT,
    INIT_ERROR,
    EMERGENCY_STOP
};

class MotionControl
{
  public:
    /**
     * Gets the a reference to a SerialControl instance.
     **/
    static MotionControl& getInstance();

    /**
     * Starts the initialization of the motion controller.
     * @return Returns true if initialization was succesfull.
     **/
    bool init();

    /**
     * Moves robot to a predefined position.
     * @param position a predefined position in the defaultPositions enum.
     **/
    void moveToPosition(defaultPositions position);

    /**
     * Moves a single servo with a specific speed.
     * @param channel	Channel the servo is connected too.
     * @param rotation	Rotation in degrees the servo has to move to.
     * @param speed		Speed in usec/s the servo will move.
     **/
    void moveServo(const uint8_t channel, const uint16_t rotation, const uint16_t speed);

    /**
     * Moves a single servo with a specific speed.
     * @param name		Name of the connected servo.
     * @param rotation	Rotation in degrees the servo has to move to.
     * @param speed		Speed in usec/s the servo will move.
     **/
    void moveServo(const std::string name, const uint16_t rotation, const uint16_t speed);

    /**
     * Moves a single servo in a spcified time.
     * @param channel	Channel the servo is connected too.
     * @param rotation	Rotation in degrees the servo has to move to.
     * @param time		Time in miliseconds the move has to take.
     **/
    void moveServoTimed(const uint8_t channel, const uint16_t rotation, const uint16_t time);
    /**
     * Moves a single servo in a spcified time.
     * @param name		Name of the connected servo.
     * @param rotation	Rotation in degrees the servo has to move to.
     * @param time		Time in miliseconds the move has to take.
     **/
    void moveServoTimed(const std::string name, const uint16_t rotation, const uint16_t time);

    /**
     * Sends a movement command that moves multiple servo's at once to a
     * location.
     * @param servoPositions An vector with pairs of a connected channel and an
     *rotation in degrees.
     * @param time		Time in miliseconds the move has to take.
     **/
    void moveMultipleServosTimed(const std::vector<std::pair<uint8_t, int16_t>> servoPositions, const uint16_t time);

    /**
     * Stops all the movement that is currently going on.
     * \# \<ch> P \<pw> \<esc> (ASCII 27)
     * @param channel	Channel the servo is connected too.
     **/
    void stopMovementCommand(const uint8_t channel);

    /**
     * Immidetly stops servo. Other servo will move futher.
     * STOP \<n>\<cr>
     * @param channel	Channel the servo is connected too.
     **/
    void stopServo(const uint8_t channel);

    /**
     * Immidetly stops servo. Other servo will move futher.
     * STOP \<n>\<cr>
     * @param name	Name of the connected servo.
     **/
    void stopServo(const std::string name);

    /**
     * Stops all connected servo's immidietly.
     **/
    void stopAllServos();

    /**
     * Checks if the last movement command has been finished.
     * Q \<cr>
     * @return Returns true if the last send movement command has been executed.
     **/
    bool isMovementFinished();

    /**
     * Gets the servo current pulse width.
     * QP \<arg> \<cr>
     * @param channel	Channel the servo is connected too.
     * @return Returns the pulse width of the servo. Returns 0 if the servo is
     *not connected.
     **/
    uint16_t getServoCurrentPulseWidth(const uint8_t channel);

    /**
     * Gets baudrate from CSS-32U. By reading a register value.
     * R4<cr>
     * @return Returns the baudrate of the controller.
     **/
    uint32_t getBaudRate();

    /**
     * Sets the bautrate of the CSS-32U.
     * R4=\<baud>
     * @param baudRate	Baudrate to set the controller to.
     **/
    void setBaudRate(uint32_t baudRate);

    /**
     * Adds a servo movement command to the end of the startup string.
     * SSCAT \#0P1500T5000?\<cr>
     * @param channel	Channel the servo is connected too.
     * @param degrees	Rotation the servo has to go too when the controller
     *starts.
     * @param speed 	Speed in usec/s the servo will move.
     **/
    void addServoStartupPosition(const uint8_t channel, const uint16_t degrees, const uint16_t speed);

    /**
     * Downloads the startup string to the controller.
     **/
    void downloadStartupString(std::string startup = "");

    /**
     * Clears the local startup string, so a new one can be constructed.
     * Use deleteStartupString() to delete it from the controller.
     **/
    void clearStartupString()
    {
        startupString = "";
    }

    /**
     * Gets the channel of a connected servo from its name.
     * @param name		Name of the servo.
     * @return			Returns the channel from the servo. If the servo
     *could
     *not
     *be found it will return 255.
     **/
    uint8_t getServoChannelFromName(std::string name);

    /**
     * Gets a reference to a connected servo.
     * @param channel	Channel the servo is connected too.
     * @return			Returns a reference to a stored Servo instance.
     **/
    const Servo& getServoFromChannel(uint8_t channel);

    /**
     * Destructor.
     **/
    virtual ~MotionControl();

    /**
     * Getter interfaceState.
     * @return Returns the current interfaceState.
     **/
    interfaceStates getInterfaceState() const
    {
        return interfaceState;
    }

    /**
     * Setter interfaceState.
     * @param interfaceState A new interfaceState.
     **/
    void setInterfaceState(interfaceStates interfaceState);

	void setWaitForResponse(bool waitForResponse) {
		this->waitForResponse = waitForResponse;
	}

  private:
    /**
     * Constructor.
     **/
    MotionControl() : interfaceState(interfaceStates::INIT), waitForResponse(true)
    {
    }

    /**
     * Moves the servo, also checks if it is possible to go to that pulsewidth.
     * # <ch> P <pw> ?S??<spd>??T?<time> <cr>
     * TODO let it check for servo max speed.
     **/
    void moveSingleServo(const uint8_t channel, const uint16_t pulseWidth, const uint16_t speed,
                         const uint16_t time = 0);

    /**
     * Checks if a given channel is supported by the SSC-32U.
     * @param channel	Channel the servo is connected too.
     * @return Returns true if the channel is accepted by the controller.
     **/
    bool isChannelValid(const uint8_t channel) const
    {
        return channel < 32;
    }

    /**
     * Checks if the given channel is connected to a servo.
     * @param channel	Channel the servo is connected too.
     * @return			Returns true if the servo is connected.
     **/
    bool isChannelConnected(const uint8_t channel) const;

    /**
     * Sets the servo center position, within a 15 degrees difference of the
     * absolute center.
     * #<ch> PO <offset value> <cr>
     * @param channel	Channel the servo is connected too.
     * @param offset	Offset in degrees.
     **/
    void setServoOffset(const uint8_t channel, const int8_t offset);

    /**
     * Deletes the startupstring.
     * SSDEL 255<cr>
     **/
    void deleteStartupString();

    /**
     * SS <cr>
     * @return Returs the start-up string stored on the controller.
     **/
    std::string getStartupString();

    /**
     * Enables the startup string on the servo controller.
     * R0=1<cr>
     * @param enabled
     **/
    void setStartupStringEnabled(bool enabled);

    /**
     * Gets a register value of the controller.
     * R<r><cr>
     * @param registerPosition	Position in the register.
     * @return					The value at the register.
     **/
    uint16_t getRegisterValue(const uint16_t registerPosition);

    /**
     * Sets a value in the controllers register.
     * R<r>=1<cr>
     * @param registerPosition	Position in the register.
     * @param value				Value to set the register value
     *to.
     **/
    void setRegisterValue(const uint16_t registerPosition, const uint16_t value);
    /**
     * Resets all register values to there default values.
     * RDFLT <cr>
     **/
    void resetRegisterValues();

    /**
     * Handles the sending of movement commands.
     **/
    void movementScheduler(std::shared_ptr<MovementCommand> command);

    interfaceStates interfaceState;

    std::vector<Servo> connectedServos;
    std::string startupString;
    SmartQueue<MovementCommand> movementCommandQueue;
    bool waitForResponse;
};

#endif /* MOTIONCONTROL_HPP_ */
