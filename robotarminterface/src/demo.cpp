/*
 * demo.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: thomas
 */

#include "ros/ros.h"

#include "robotarminterface/emergencyStopMsg.h"
#include "robotarminterface/interfaceStateSrv.h"
#include "robotarminterface/moveServo.h"
#include "robotarminterface/moveServoDefinedMsg.h"
#include "robotarminterface/moveSingleServoMsg.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_app");

    ros::NodeHandle n;

    ros::Publisher definedServoPub =
        n.advertise<robotarminterface::moveServoDefinedMsg>(
            "moveServoToDefinedPosition", 1000);
    ros::Publisher servoPosPub =
        n.advertise<robotarminterface::moveSingleServoMsg>("moveSingleServo",
                                                           1000);
    ros::Publisher emergencyStopPub =
        n.advertise<robotarminterface::emergencyStopMsg>("emergencyStop", 1000);

    std::cout << "Waiting 5 seconds." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    robotarminterface::moveServoDefinedMsg msg;
    msg.position = "STRAIGHT_UP";

    definedServoPub.publish(msg);
    ros::spinOnce();
    msg.position = "PARK";

    std::cout << "Moving to Straight up." << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(4));

    std::array<unsigned char, 5> servoChannels = {{0, 1, 2, 3, 5}};

    robotarminterface::moveSingleServoMsg moveMsg;
    robotarminterface::moveServo servo;
    moveMsg.time = 2500;

    definedServoPub.publish(msg);
    ros::spinOnce();

    servo.channel = 0;
    servo.rotation = -90;
    moveMsg.servo = servo;
    servoPosPub.publish(moveMsg);
    ros::spinOnce();

    servo.channel = 0;
    servo.rotation = 90;
    moveMsg.servo = servo;
    servoPosPub.publish(moveMsg);
    ros::spinOnce();

    definedServoPub.publish(msg);
    ros::spinOnce();

    servo.channel = 2;
    servo.rotation = 0;
    moveMsg.servo = servo;
    servoPosPub.publish(moveMsg);
    ros::spinOnce();

    servo.channel = 2;
    servo.rotation = 70;
    moveMsg.servo = servo;
    servoPosPub.publish(moveMsg);
    ros::spinOnce();

    definedServoPub.publish(msg);
    ros::spinOnce();

    std::cout << "Moving arms to random positions." << std::endl;

    std::cout << "Waiting 15 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));

    robotarminterface::emergencyStopMsg esMsg;
    esMsg.stop = true;

    emergencyStopPub.publish(esMsg);
    ros::spinOnce();
    std::cout << "EMERGENCY STOP" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(4));

    msg.position = "PARK";
    definedServoPub.publish(msg);
    ros::spinOnce();

    std::cout << "TRYING TO MOVE TO PARK." << std::endl;

    char a;
    std::cin >> a;
}
