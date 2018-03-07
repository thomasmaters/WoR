/*
 * RosCommunication.cpp
 *
 *  Created on: Mar 1, 2017
 *      Author: Thomas Maters
 */

#include "RosCommunication.hpp"
#include <boost/algorithm/string.hpp>

RosCommunication::RosCommunication()
{
}

RosCommunication& RosCommunication::getInstance()
{
    static RosCommunication instance;
    return instance;
}

bool RosCommunication::init()
{
    try
    {
        interfaceStateService =
            serviceNode.advertiseService("interfaceState", &RosCommunication::getInterfaceStateRequest, this);
        moveServoToDefinedPositionSubscriber = subsriberNode.subscribe(
            "moveServoToDefinedPosition", 10, &RosCommunication::moveServoToDefinedPosCallback, this);
        moveSingleServoSubscriber =
            subsriberNode.subscribe("moveSingleServo", 10, &RosCommunication::moveSingleServoCallback, this);
        moveMultipleServosSubscriber =
            subsriberNode.subscribe("moveMultipleServos", 10, &RosCommunication::moveMultipleServosCallback, this);
        emergencyStopSubscriber =
            subsriberNode.subscribe("emergencyStop", 10, &RosCommunication::emergencyStopCallback, this);
        ssc32uCommandoPublisher = publisherNode.advertise<robotsimulation::ssc32u_command>("ssc32u_topic", 10);
    }
    catch (...)
    {
        return false;
    }

    return true;
}

bool RosCommunication::getInterfaceStateRequest(robotarminterface::interfaceStateSrv::Request& req,
                                                robotarminterface::interfaceStateSrv::Response& res)
{
    res.state = MotionControl::getInstance().getInterfaceState();
    return true;
}

void RosCommunication::moveServoToDefinedPosCallback(const robotarminterface::moveServoDefinedMsg& msg)
{
    std::string pos = boost::to_upper_copy<std::string>(msg.position);

    if (pos == "READY")
    {
        MotionControl::getInstance().moveToPosition(defaultPositions::READY);
    }
    else if (pos == "PARK")
    {
        MotionControl::getInstance().moveToPosition(defaultPositions::PARK);
    }
    else if (pos == "STRAIGHT_UP")
    {
        MotionControl::getInstance().moveToPosition(defaultPositions::STRAIGHT_UP);
    }
}

void RosCommunication::emergencyStopCallback(const robotarminterface::emergencyStopMsg& msg)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    MotionControl::getInstance().setInterfaceState(interfaceStates::EMERGENCY_STOP);
    MotionControl::getInstance().stopAllServos();
}

void RosCommunication::moveSingleServoCallback(const robotarminterface::moveSingleServoMsg& msg)
{
    MotionControl::getInstance().moveServoTimed(msg.servo.channel, msg.servo.rotation, msg.time);
}

void RosCommunication::moveMultipleServosCallback(const robotarminterface::moveMultipleServosMsg& msg)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::pair<uint8_t, int16_t> temp;
    std::vector<std::pair<uint8_t, int16_t>> tempVector;

    for (robotarminterface::moveServo servoCommand : msg.servos)
    {
        temp.first = servoCommand.channel;
        temp.second = servoCommand.rotation;
        tempVector.push_back(temp);
    }

    MotionControl::getInstance().moveMultipleServosTimed(tempVector, msg.time);
}

void RosCommunication::sendSsc32uCommand(const std::string command)
{
    robotsimulation::ssc32u_command msg;
    msg.command = command;
    ssc32uCommandoPublisher.publish(msg);
    ros::spinOnce();
}

std::vector<Servo> RosCommunication::getRosParamServoConfiguration() const
{
    try
    {
        std::vector<Servo> connected_servos;
        XmlRpc::XmlRpcValue servo_config;
        subsriberNode.getParam("servos", servo_config);

        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = servo_config.begin(); it != servo_config.end();
             it++)
        {
            if (it->second.size() != 10)
            {
                throw std::runtime_error("Invalid parameter when trying to "
                                         "load servo configuration");
            }
            connected_servos.push_back(Servo(static_cast<int>(it->second[0]), static_cast<std::string>(it->second[1]),
                                             static_cast<int>(it->second[2]), static_cast<int>(it->second[3]),
                                             static_cast<int>(it->second[4]), static_cast<int>(it->second[5]),
                                             static_cast<int>(it->second[6]), static_cast<int>(it->second[7]),
                                             static_cast<double>(it->second[8]), static_cast<bool>(it->second[9])));
        }

        return connected_servos;
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    throw std::runtime_error("Something failed when getting the servo "
                             "configuration from the ros param server.");
}

RosCommunication::~RosCommunication()
{
    // TODO Auto-generated destructor stub
}
