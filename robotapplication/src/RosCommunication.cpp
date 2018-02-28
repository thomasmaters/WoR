/*
 * RosCommunication.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#include "RosCommunication.h"
#include <thread>

RosCommunication::RosCommunication()
{
    target_position_subscriber_ =
        subsriber_node_.subscribe("pickupTarget", 10, &RosCommunication::moveToTargetCallback, this);
    multipleArmPositions = publisher_node_.advertise<robotarminterface::moveMultipleServosMsg>("moveMultipleServos", 10);
    singleArmPosition = publisher_node_.advertise<robotarminterface::moveSingleServoMsg>("moveSingleServo", 10);
    definedArmPosition =
        publisher_node_.advertise<robotarminterface::moveServoDefinedMsg>("moveServoToDefinedPosition", 10);

    phis = { 0.1, InverseKinematics::toRadians(90.0), InverseKinematics::toRadians(-90.0) };
    lastPhis = phis;

    std::cout << __PRETTY_FUNCTION__ << std::endl;
    ros::spin();
}

void RosCommunication::moveToTargetCallback(const robotapplication::PickAndPlace msg)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    moveToPos(msg.target.x / 10, msg.target.y / 10, (msg.target.z + 20) / 10, msg.targetRotation);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    moveToPos(msg.target.x / 10, msg.target.y / 10, (msg.target.z - 20) / 10, msg.targetRotation);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    moveToPos(msg.target.x / 10, msg.target.y / 10, (msg.target.z - 20) / 10, msg.targetRotation, true);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    moveToPos(msg.target.x / 10, msg.target.y / 10, (msg.target.z + 40) / 10, msg.targetRotation, true);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    moveToPos(msg.dest.x / 10, msg.dest.y / 10, (msg.dest.z + 20) / 10, msg.destRotation, true);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    moveToPos(msg.dest.x / 10, msg.dest.y / 10, (msg.dest.z + 20) / 10, msg.destRotation);

    std::this_thread::sleep_for(std::chrono::seconds(40));
    robotarminterface::moveServoDefinedMsg moveServoDefinedMsg;
    moveServoDefinedMsg.position = "PARK";
    definedArmPosition.publish(moveServoDefinedMsg);
    // std::this_thread::sleep_for(std::chrono::seconds(4));
    // moveToPos(msg.dest.x / 10, msg.dest.y / 10, (msg.dest.z + 12) / 10);
    // std::this_thread::sleep_for(std::chrono::seconds(4));
    // moveToPos(msg.dest.x / 10, msg.dest.y / 10, (msg.dest.z + 6) / 10);
}

void RosCommunication::moveToPos(const float y, const float x, const float z, const float rotation,
                                 const bool gripperState)
{
    bool inverseRotation = y < 0;
    Matrix<1, 3, double> goal = { x, std::abs(y), z };
    std::cout << "goal: " << goal << std::endl;
    lastPhis = phis;
    InverseKinematics::getInstance().calculateInverse(goal, lastPhis, armLengths);
    if (!inverseRotation)
    {
        lastPhis[0][0] = -lastPhis[0][0] - InverseKinematics::toRadians(15);
    }
    else
    {
        lastPhis[0][0] -= InverseKinematics::toRadians(15);
    }

    robotarminterface::moveMultipleServosMsg moveServosMsg;
    robotarminterface::moveServo moveSingleServoMsg;
    for (int i = 0; i < phis.getHeight(); ++i)
    {
        moveSingleServoMsg.channel = i;
        moveSingleServoMsg.rotation = InverseKinematics::toDegrees(lastPhis.at(i, 0));
        std::cout << "i: " << i << " rot: " << InverseKinematics::toDegrees(lastPhis.at(i, 0)) << std::endl;
        moveServosMsg.servos.push_back(moveSingleServoMsg);
    }
    moveSingleServoMsg.channel = 3;
    moveSingleServoMsg.rotation =
        -90 - InverseKinematics::toDegrees(lastPhis.at(1, 0)) - InverseKinematics::toDegrees(lastPhis.at(2, 0));
    moveServosMsg.servos.push_back(moveSingleServoMsg);

    moveSingleServoMsg.channel = 4;
    moveSingleServoMsg.rotation = gripperState ? 90 : -90;
    moveServosMsg.servos.push_back(moveSingleServoMsg);

    moveSingleServoMsg.channel = 5;
    moveSingleServoMsg.rotation = rotation;
    moveServosMsg.servos.push_back(moveSingleServoMsg);

    moveServosMsg.time = 4000;
    multipleArmPositions.publish(moveServosMsg);
}

RosCommunication::~RosCommunication()
{
    // TODO Auto-generated destructor stub
}
