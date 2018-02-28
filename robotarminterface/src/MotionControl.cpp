/*
 * MotionControl.cpp
 *
 *  Created on: 18 feb. 2017
 *      Author: Thomas
 */

#include "MotionControl.hpp"
#include "SerialControl.hpp"
#include <thread>

MotionControl& MotionControl::getInstance()
{
    static MotionControl instance;
    return instance;
}

bool MotionControl::init()
{
	try {
		startupString = "";

		//connectedServos.push_back(
		//	Servo(0, "base", -95, 95, 553, 2425, -90, 90, 0.22));
		//connectedServos.push_back(
		//	Servo(1, "shoulder", -99, 99, 556, 2420, -30, 90, 0.19, true));
		//connectedServos.push_back(
		//	Servo(2, "elbow", 0, 202, 556, 2410 , 0, 135, 0.28));
		//connectedServos.push_back(
		//	Servo(3, "wrist", -93, 93, 553, 2520, -90, 90, 0.24,true));
		//connectedServos.push_back(
		//	Servo(5, "wrist rotate", -91, 90, 553, 2300, -90, 90, 0.16));

		/*connectedServos.push_back(
			Servo(0, "base", -90, 90, 640, 2360,-90, 90, 0.22,true));
		connectedServos.push_back(
			Servo(1, "shoulder", 0, 120, 800, 1800, 0, 120, 0.19));
		connectedServos.push_back(
			Servo(2, "elbow", -135, 0, 680, 1920 , -135, 0, 0.28,true));
		connectedServos.push_back(
			Servo(3, "wrist", -90, 90, 630, 2430, -90, 90, 0.24));
		connectedServos.push_back(
			Servo(4, "wrist_rotate", -90, 90, 700, 2400, -90, 90, 0.16));
		connectedServos.push_back(
			Servo(5, "gripper", -90, 90, 700, 2400, -90, 90, 0.16));*/

		connectedServos.push_back(
			Servo(0, "base", -90, 90, 640, 2360,-90, 90, 0.22,true));
		connectedServos.push_back(
			Servo(1, "shoulder", 0, 120, 500, 1900, 0, 120, 0.19));
		connectedServos.push_back(
			Servo(2, "elbow", -135, 0, 700, 1920 , -135, 0, 0.28,true));
		connectedServos.push_back(
			Servo(3, "wrist", -90, 90, 500, 2500, -90, 90, 0.24));
		connectedServos.push_back(
			Servo(4, "gripper", -90, 90, 700, 2400, -90, 90, 0.16));
		connectedServos.push_back(
			Servo(5, "wrist_rotate", -90, 90, 700, 2400, -90, 90, 0.16));

		movementCommandQueue.setCallbackFunction(
			[this](std::shared_ptr<MovementCommand> command) {
				MotionControl::movementScheduler(command);
			});
	} catch (...) {
		return false;
	}
	return true;
}

// TODO posities.
void MotionControl::moveToPosition(defaultPositions position)
{
	if(interfaceState != interfaceStates::EMERGENCY_STOP)
	{
		switch (position)
		{
		case defaultPositions::PARK:
			movementScheduler(
					std::shared_ptr<MovementCommand>(
							new MovementCommand("#0P1500#1P1970#2P1795#3P500#5P1527T3000\r")));
			break;
		case defaultPositions::READY:
			movementScheduler(
					std::shared_ptr<MovementCommand>(
							new MovementCommand("#0P1500#1P1676#2P1473#3P1430#5P1527T3000\r")));
			break;
		case defaultPositions::STRAIGHT_UP:
			movementScheduler(
					std::shared_ptr<MovementCommand>(
							new MovementCommand("#0P1500#1P1582#2P693#3P1536#5P1527T3000\r")));
			break;
		default:
			break;
		}
	}
}

/**************************************************
 * Public signle servo movement commands.**********
 **************************************************/
void MotionControl::moveServo(const uint8_t channel, const uint16_t rotation,
                              const uint16_t speed)
{
    if (!isChannelConnected(channel)) {
        return;
    }

    uint16_t pulseWidth =
        getServoFromChannel(channel).getPulseWidthFromDegrees(rotation);
    std::cout << "calculated pulse width " << pulseWidth << std::endl;

    moveSingleServo(channel, pulseWidth, speed, 0);
}

void MotionControl::moveServo(const std::string name, const uint16_t rotation,
                              const uint16_t speed)
{
    moveServo(getServoChannelFromName(name), rotation, speed);
}

void MotionControl::moveServoTimed(const uint8_t channel,
                                   const uint16_t rotation, const uint16_t time)
{
    if (!isChannelConnected(channel)) {
        return;
    }

    uint16_t pulseWidth = getServoFromChannel(channel).getPulseWidthFromDegrees(rotation);

    moveSingleServo(channel, pulseWidth, 0, time);
}

void MotionControl::moveServoTimed(const std::string name,
                                   const uint16_t rotation, const uint16_t time)
{
    moveServoTimed(getServoChannelFromName(name), rotation, time);
}

/**************************************************
 **************Misc functions**********************
 **************************************************/

void MotionControl::moveMultipleServosTimed(
    const std::vector<std::pair<uint8_t, int16_t>> servoPositions,
    const uint16_t time)
{
	if(interfaceState != interfaceStates::EMERGENCY_STOP)
	{
		std::string command = "";
		for (std::pair<uint8_t, int16_t> movement : servoPositions) {
			if (isChannelConnected(movement.first)) {
				std::cout << std::to_string(movement.first) << "," << std::to_string(movement.second) << "," << std::to_string(	getServoFromChannel(movement.first).getPulseWidthFromDegrees(movement.second)) << std::endl;
				
				command +=
					"#" + std::to_string(movement.first) + "P" +
					std::to_string(
						getServoFromChannel(movement.first).getPulseWidthFromDegrees(
							movement.second));
			}
		}
		command += "T" + std::to_string(time) + "\r";
		std::cout << "MoveMultipleServosCommand: " << command << std::endl;
		movementCommandQueue.addToQueue(
			std::shared_ptr<MovementCommand>(new MovementCommand(command)));
		if(interfaceState == interfaceStates::IDLE)
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
    for (const Servo servo : connectedServos) {
        stopServo(servo.getChannel());
    }
}

void MotionControl::stopServo(const uint8_t channel)
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
    if (!isChannelConnected(channel)) {
        return;
    }
    std::cout << "Trying to stop servo: " << std::to_string(channel) << std::endl;
    SerialControl::getInstance().write("STOP" + std::to_string(channel) + '\r');
}

void MotionControl::stopServo(const std::string name)
{
    uint8_t channel = getServoChannelFromName(name);
    stopServo(channel);
}

bool MotionControl::isMovementFinished()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::string returnValue =
        SerialControl::getInstance().writeAndRead("Q\r", 1);
    std::cout << "isMovementFinished: " << returnValue << std::endl;
    return returnValue == ".";
}

uint16_t MotionControl::getServoCurrentPulseWidth(const uint8_t channel)
{
    if (!isChannelConnected(channel)) {
        return 0;
    }

    return SerialControl::getInstance().writeAndReadUntilAsInt(
        "QP" + std::to_string(channel), '\0');
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

void MotionControl::addServoStartupPosition(const uint8_t channel,
                                            const uint16_t degrees,
                                            const uint16_t speed)
{
    if (!isChannelConnected(channel) ||
        !connectedServos[channel].canMoveToDegrees(degrees)) {
        return;
    }

    // TODO speed variable
    startupString +=
        "#" + std::to_string(channel) + "P" +
        std::to_string(
            connectedServos[channel].getPulseWidthFromDegrees(degrees)) +
        ";";
}

void MotionControl::downloadStartupString(std::string startup)
{
    deleteStartupString();
    if(startup != "")
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
    for (const Servo servo : connectedServos) {
        if (servo.getName() == name) {
            return servo.getChannel();
        }
    }
    return 255;
}

const Servo& MotionControl::getServoFromChannel(uint8_t channel)
{
    for (const Servo& servo : connectedServos) {
        if (servo.getChannel() == channel) {
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

void MotionControl::moveSingleServo(const uint8_t channel,
                                    const uint16_t pulseWidth,
                                    const uint16_t speed, const uint16_t time)
{
    std::cout << __PRETTY_FUNCTION__ << pulseWidth << std::endl;
    if (isChannelConnected(channel) &&
        getServoFromChannel(channel).canMoveToPulseWidth(pulseWidth) && interfaceState != interfaceStates::EMERGENCY_STOP) {
        movementCommandQueue.addToQueue(std::shared_ptr<MovementCommand>(
            new MovementCommand(channel, pulseWidth, speed, time)));
        if(interfaceState == interfaceStates::IDLE)
        {
        	movementCommandQueue.notifyEventLoop();
        }
    }
    else
    {
    	std::cout << "Can't move to pulsewidth or servo is not connected." << std::endl;
    }
}

bool MotionControl::isChannelConnected(const uint8_t channel) const
{
    if (!isChannelValid(channel)) {
        std::cout << "CHANNEL OUT OF RANGE" << std::endl;
        return false;
    }

    for (const Servo servo : connectedServos) {
        if (servo.getChannel() == channel) {
            return true;
        }
    }
    std::cout << "CHANNEL NOT CONNECTED" << std::endl;
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
    return SerialControl::getInstance().writeAndReadUntil("SS\r", '\r');
}

void MotionControl::setStartupStringEnabled(bool enabled)
{
    setRegisterValue(0, enabled);
}

uint16_t MotionControl::getRegisterValue(const uint16_t registerPosition)
{
    return SerialControl::getInstance().writeAndReadUntilAsInt(
        "R" + std::to_string(registerPosition) + '\r', '\r');
}

void MotionControl::setRegisterValue(const uint16_t registerPosition,
                                     const uint16_t value)
{
    SerialControl::getInstance().write("R" + std::to_string(registerPosition) +
                                       "=" + std::to_string(value) + '\r');
}

void MotionControl::resetRegisterValues()
{
    SerialControl::getInstance().write("RDFLT\r");
}

void MotionControl::movementScheduler(std::shared_ptr<MovementCommand> command)
{
	setInterfaceState(interfaceStates::SENDING_COMMAND);
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::string commandString = command->getCommandString();
    std::cout << commandString << std::endl;
    SerialControl::getInstance().write(commandString);
    RosCommunication::getInstance().sendSsc32uCommand(commandString);

    command.reset();
    boost::thread t([this]() {
    	setInterfaceState(interfaceStates::MOVING_ARM);
        while (!isMovementFinished()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(interfaceState == interfaceStates::EMERGENCY_STOP)
			{
				return;
			}
        }
        setInterfaceState(interfaceStates::MOVING_FINISHED);

        if(movementCommandQueue.size() == 0)
		{
        	setInterfaceState(interfaceStates::IDLE);
		}

        movementCommandQueue.notifyEventLoop();

        std::cout << "MOTION DONE" << std::endl;
    });
}
