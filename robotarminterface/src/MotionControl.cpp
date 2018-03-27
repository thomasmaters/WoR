/*
 * MotionControl.cpp
 *
 *  Created on: 18 feb. 2017
 *      Author: Thomas
 */

#include "MotionControl.hpp"
#include <thread>
#include "SerialControl.hpp"

MotionControl& MotionControl::getInstance()
{
    static MotionControl instance;
    return instance;
}

bool MotionControl::init()
{
    try
    {
        startupString = "";

        connectedServos = RosCommunication::getInstance().getRosParamServoConfiguration();

        movementCommandQueue.setCallbackFunction(
            [this](std::shared_ptr<MovementCommand> command) { MotionControl::movementScheduler(command); });
    }
    catch (...)
    {
        return false;
    }
    return true;
}

void MotionControl::moveToPosition(defaultPositions position)
{
    if (interfaceState != interfaceStates::EMERGENCY_STOP)
    {
        switch (position)
        {
            case defaultPositions::PARK:
                movementScheduler(
                    std::shared_ptr<MovementCommand>(new MovementCommand("#0P1500#1P1900#2P1790#3P500#5P1527T3000\r")));
                break;
            case defaultPositions::READY:
                movementScheduler(std::shared_ptr<MovementCommand>(
                    new MovementCommand("#0P1500#1P1676#2P1473#3P1430#5P1527T3000\r")));
                break;
            case defaultPositions::STRAIGHT_UP:
                movementScheduler(
                    std::shared_ptr<MovementCommand>(new MovementCommand("#0P1500#1P1582#2P693#3P1536#5P1527T3000\r")));
                break;
            default:
                break;
        }
    }
}

/**************************************************
 * Public signle servo movement commands.**********
 **************************************************/
void MotionControl::moveServo(const uint8_t channel, const uint16_t rotation, const uint16_t speed)
{
    if (!isChannelConnected(channel))
    {
        return;
    }

    uint16_t pulseWidth = getServoFromChannel(channel).getPulseWidthFromDegrees(rotation);

    moveSingleServo(channel, pulseWidth, speed, 0);
}

void MotionControl::moveServo(const std::string name, const uint16_t rotation, const uint16_t speed)
{
    moveServo(getServoChannelFromName(name), rotation, speed);
}

void MotionControl::moveServoTimed(const uint8_t channel, const uint16_t rotation, const uint16_t time)
{
    if (!isChannelConnected(channel))
    {
        return;
    }

    uint16_t pulseWidth = getServoFromChannel(channel).getPulseWidthFromDegrees(rotation);

    moveSingleServo(channel, pulseWidth, 0, time);
}

void MotionControl::moveServoTimed(const std::string name, const uint16_t rotation, const uint16_t time)
{
    moveServoTimed(getServoChannelFromName(name), rotation, time);
}

/**************************************************
 **************Misc functions**********************
 **************************************************/

void MotionControl::moveMultipleServosTimed(const std::vector<std::pair<uint8_t, int16_t>> servoPositions,
                                            const uint16_t time)
{
    if (interfaceState != interfaceStates::EMERGENCY_STOP)
    {
        std::string command = "";
        for (std::pair<uint8_t, int16_t> movement : servoPositions)
        {
            if (isChannelConnected(movement.first))
            {
                ROS_DEBUG_STREAM(
                    std::to_string(movement.first)
                    << "," << std::to_string(movement.second) << ","
                    << std::to_string(getServoFromChannel(movement.first).getPulseWidthFromDegrees(movement.second)));

                command +=
                    "#" + std::to_string(movement.first) + "P" +
                    std::to_string(getServoFromChannel(movement.first).getPulseWidthFromDegrees(movement.second));
            }
        }
        command += "T" + std::to_string(time) + "\r";
        ROS_DEBUG_STREAM("MoveMultipleServosCommand: " << command);
        movementCommandQueue.addToQueue(std::shared_ptr<MovementCommand>(new MovementCommand(command)));
        if (interfaceState == interfaceStates::IDLE)
        {
            movementCommandQueue.notifyEventLoop();
        }
    }
}

// TODO hier na kijken hoe die commando precies geimplementeerd moet worden.
void MotionControl::stopMovementCommand(const uint8_t channel)
{
}

void MotionControl::stopAllServos()
{
    movementCommandQueue.clearQueue();
    for (const Servo servo : connectedServos)
    {
        stopServo(servo.getChannel());
    }
}

void MotionControl::stopServo(const uint8_t channel)
{
    if (!isChannelConnected(channel))
    {
        return;
    }
    ROS_DEBUG_STREAM("Trying to stop servo: " << std::to_string(channel));
    SerialControl::getInstance().write("STOP" + std::to_string(channel) + '\r');
}

void MotionControl::stopServo(const std::string name)
{
    uint8_t channel = getServoChannelFromName(name);
    stopServo(channel);
}

bool MotionControl::isMovementFinished()
{
    if (waitForResponse)
    {
        std::string returnValue = SerialControl::getInstance().writeAndRead("Q\r", 1);
        return returnValue == ".";
    }
    return false;
}

uint16_t MotionControl::getServoCurrentPulseWidth(const uint8_t channel)
{
    if (!isChannelConnected(channel) || !waitForResponse)
    {
        return 0;
    }

    return SerialControl::getInstance().writeAndReadUntilAsInt("QP" + std::to_string(channel), '\0');
}

uint32_t MotionControl::getBaudRate()
{
    uint16_t baudRate = getRegisterValue(4);
    return baudRate * 10;
}

void MotionControl::setBaudRate(uint32_t baudRate)
{
    setRegisterValue(4, baudRate / 10);
}

void MotionControl::addServoStartupPosition(const uint8_t channel, const uint16_t degrees, const uint16_t speed)
{
    if (!isChannelConnected(channel) || !connectedServos[channel].canMoveToDegrees(degrees))
    {
        return;
    }

    // TODO speed variable
    startupString += "#" + std::to_string(channel) + "P" +
                     std::to_string(connectedServos[channel].getPulseWidthFromDegrees(degrees)) + ";";
}

void MotionControl::downloadStartupString(std::string startup)
{
    deleteStartupString();
    if (startup != "")
    {
        SerialControl::getInstance().write("SSCAT " + startup + '\r');
    }
    else
    {
        SerialControl::getInstance().write("SSCAT" + startupString + '\r');
    }
}

uint8_t MotionControl::getServoChannelFromName(std::string name)
{
    for (const Servo servo : connectedServos)
    {
        if (servo.getName() == name)
        {
            return servo.getChannel();
        }
    }
    return 255;
}

const Servo& MotionControl::getServoFromChannel(uint8_t channel)
{
    for (const Servo& servo : connectedServos)
    {
        if (servo.getChannel() == channel)
        {
            return servo;
        }
    }
    throw "SERVO NOT CONNECTED";
}

MotionControl::~MotionControl()
{
}

/**************************************************
 ******************Private functions***************
 **************************************************/

void MotionControl::moveSingleServo(const uint8_t channel, const uint16_t pulseWidth, const uint16_t speed,
                                    const uint16_t time)
{
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << pulseWidth);
    if (isChannelConnected(channel) && getServoFromChannel(channel).canMoveToPulseWidth(pulseWidth) &&
        interfaceState != interfaceStates::EMERGENCY_STOP)
    {
        movementCommandQueue.addToQueue(
            std::shared_ptr<MovementCommand>(new MovementCommand(channel, pulseWidth, speed, time)));
        if (interfaceState == interfaceStates::IDLE)
        {
            movementCommandQueue.notifyEventLoop();
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't move to pulsewidth or servo is not connected.");
    }
}

bool MotionControl::isChannelConnected(const uint8_t channel) const
{
    if (!isChannelValid(channel))
    {
        return false;
    }

    for (const Servo servo : connectedServos)
    {
        if (servo.getChannel() == channel)
        {
            return true;
        }
    }
    return false;
}

void MotionControl::setServoOffset(const uint8_t channel, const int8_t offset)
{
    SerialControl::getInstance().write("SSDEL 255\r");
}

void MotionControl::deleteStartupString()
{
    SerialControl::getInstance().write("SSDEL 255\r");
}

std::string MotionControl::getStartupString()
{
    if (waitForResponse)
    {
        return SerialControl::getInstance().writeAndReadUntil("SS\r", '\r');
    }
    return "";
}

void MotionControl::setStartupStringEnabled(bool enabled)
{
    setRegisterValue(0, enabled);
}

uint16_t MotionControl::getRegisterValue(const uint16_t registerPosition)
{
    if (waitForResponse)
    {
        return SerialControl::getInstance().writeAndReadUntilAsInt("R" + std::to_string(registerPosition) + '\r', '\r');
    }
    return 0;
}

void MotionControl::setRegisterValue(const uint16_t registerPosition, const uint16_t value)
{
    SerialControl::getInstance().write("R" + std::to_string(registerPosition) + "=" + std::to_string(value) + '\r');
}

void MotionControl::resetRegisterValues()
{
    SerialControl::getInstance().write("RDFLT\r");
}

void MotionControl::setInterfaceState(interfaceStates interfaceState)
{
    ROS_INFO_STREAM("Interface state set to: " << interfaceState);
    this->interfaceState = interfaceState;
}

void MotionControl::movementScheduler(std::shared_ptr<MovementCommand> command)
{
    setInterfaceState(interfaceStates::SENDING_COMMAND);

    std::string commandString = command->getCommandString();
    uint16_t movementDuration = command->getTime();

    SerialControl::getInstance().write(commandString);
    RosCommunication::getInstance().sendSsc32uCommand(commandString);

    command.reset();
    boost::thread t([this, movementDuration]() {
        setInterfaceState(interfaceStates::MOVING_ARM);
        if (waitForResponse)
        {
            while (!isMovementFinished())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (interfaceState == interfaceStates::EMERGENCY_STOP)
                {
                    return;
                }
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(movementDuration));
        }

        setInterfaceState(interfaceStates::MOVING_FINISHED);

        if (movementCommandQueue.size() == 0)
        {
            setInterfaceState(interfaceStates::IDLE);
        }

        movementCommandQueue.notifyEventLoop();

        ROS_INFO("Movement command handled.");
    });
}
