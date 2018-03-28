/*
 * RosCommunication.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#include "motion_control_controller.h"

#include <thread>

MotionControl::MotionControl() : MotionControlInterface()
{
    phis = { 0.1, InverseKinematics::toRadians(90.0), InverseKinematics::toRadians(-90.0) };
    lastPhis = phis;
    armLengths = getArmLenghtConfiguration();
    solutionSpace = getServoConfigurationSpace();

    std::cout << "Armlengths: " << armLengths << std::endl;
    std::cout << "SolutionSpace: " << solutionSpace << std::endl;
    ros::spin();
}

void MotionControl::moveToTarget(const robotapplication::pick_and_place msg)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    moveToPos(msg.target.x, msg.target.y, (msg.target.z + 20), msg.targetRotation);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    moveToPos(msg.target.x, msg.target.y, (msg.target.z - 20), msg.targetRotation);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    moveToPos(msg.target.x, msg.target.y, (msg.target.z - 20), msg.targetRotation, true);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    moveToPos(msg.target.x, msg.target.y, (msg.target.z + 40), msg.targetRotation, true);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    moveToPos(msg.dest.x, msg.dest.y, (msg.dest.z + 20), msg.destRotation, true);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    moveToPos(msg.dest.x, msg.dest.y, (msg.dest.z + 20), msg.destRotation);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    robotarminterface::moveServoDefinedMsg moveServoDefinedMsg;
    moveServoDefinedMsg.position = "PARK";
    publishDefinedServoMsg(moveServoDefinedMsg);
}

void MotionControl::moveToPos(const float y, const float x, const float z, const float rotation,
                              const bool gripperState)
{
    bool inverseRotation = y < 0;
    Matrix<3, 1, double> goal = { x, y, z };
    std::cout << "goal: " << goal << std::endl;
    lastPhis = phis;
    InverseKinematics::getInstance().calculateInverse(goal, lastPhis, armLengths, solutionSpace);

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
    moveSingleServoMsg.rotation = rotation;
    moveServosMsg.servos.push_back(moveSingleServoMsg);

    moveSingleServoMsg.channel = 5;
    moveSingleServoMsg.rotation = gripperState ? 90 : -90;
    moveServosMsg.servos.push_back(moveSingleServoMsg);

    moveServosMsg.time = 4000;
    publishMultipleServoMsg(moveServosMsg);
}

MotionControl::~MotionControl()
{
    // TODO Auto-generated destructor stub
}
