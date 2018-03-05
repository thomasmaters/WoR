/*
 * ssc32u_parser_to_jointstate.cpp
 *
 *  Created on: Mar 2, 2018
 *      Author: Thomas Maters
 */

#include "ssc32u_parser_to_jointstate.h"

#include <cmath>
#include <regex>
#include <thread>

JointStateConverter::JointStateConverter() : at_destination_(false), stopping_(false)
{
    joint_state_msg_.name.resize(connected_servos_.size());
    joint_state_msg_.position.resize(connected_servos_.size());
    joint_state_msg_.name[0] = "base_link2turret";
    joint_state_msg_.name[1] = "turret2upperarm";
    joint_state_msg_.name[2] = "upperarm2forearm";
    joint_state_msg_.name[5] = "wrist2hand";
    joint_state_msg_.name[4] = "gripper_left2hand";
    joint_state_msg_.name[3] = "forearm2wrist";

    current_positions_[0] = 1500;
    current_positions_[1] = 1500;
    current_positions_[2] = 1500;
    current_positions_[3] = 1500;
    current_positions_[4] = 1500;
    current_positions_[5] = 1500;
}

void JointStateConverter::constructJointStateMessage()
{
    for (const std::pair<const uint8_t, double>& servo_item : current_positions_)
    {
        // TODO Joinstate messages houden geen rekening met het type van de joint in de simulatie. Dit geeft dus
        // problemen bij de grijper omdat die in meters is.
        joint_state_msg_.position[servo_item.first] =
            (connected_servos_[servo_item.first].getDegreesFromPulseWidth(static_cast<int16_t>(servo_item.second)) *
             M_PI) /
            180;
    }
    joint_state_msg_.header.stamp = ros::Time::now();
    sendJointStateMessage(joint_state_msg_);
}

void JointStateConverter::moveToDesitnation(double movement_time,
                                            const std::map<int16_t, std::pair<int16_t, int16_t>>& desired_positions)
{
    ros::Rate rate(FPS);
    // Move slowly to the destination.
    while (movement_time > 0)
    {
        for (const std::pair<const int16_t, std::pair<int16_t, int16_t>>& servo_item : desired_positions)
        {
            double current_increase = 0;
            double pwm_to_go = 0;
            // Does the servo have a defined speed to move?
            if (servo_item.second.second != 0)
            {
                pwm_to_go = std::floor((double)current_positions_[servo_item.first] - (double)servo_item.second.first);

                if (abs(pwm_to_go) > servo_item.second.second / FPS)
                {
                    current_increase = servo_item.second.second / FPS * (pwm_to_go < 0 ? 1 : -1);
                }
                else
                {
                    current_increase = -pwm_to_go;
                }
            }
            else if (movement_time > 0)  // Do we still have time left
            {
                pwm_to_go = (double)current_positions_[servo_item.first] - (double)servo_item.second.first;
                current_increase = pwm_to_go / movement_time / FPS * -1000;
            }

            current_positions_[servo_item.first] += current_increase;
            if (current_positions_[servo_item.first] < connected_servos_[servo_item.first].getMinPulseWidth())
            {
                current_positions_[servo_item.first] = connected_servos_[servo_item.first].getMinPulseWidth();
            }
        }

        constructJointStateMessage();
        // Decrease time left.
        movement_time -= 1000 / FPS;
    }
    at_destination_ = true;
}

void JointStateConverter::parseSsc32uCommand(const robotsimulation::ssc32u_command& msg)
{
    //<servoNumber, pair<toAngle, speed>>
    std::map<int16_t, std::pair<int16_t, int16_t>> desired_positions;
    std::regex regex_1("#(\\d*)P(\\d*)S(\\d*)");
    std::regex regex_2("#(\\d*)P(\\d*)");
    std::regex regex_3("T(\\d*)");
    std::smatch match;
    std::string command = msg.command;
    double movement_time = std::string::npos;

    ROS_INFO_STREAM("Trying to parse command: " << command);

    // Parse command.
    while (std::regex_search(command, match, regex_1))
    {
        desired_positions[std::stoi(match[1])] = std::make_pair(std::stoi(match[2]), std::stoi(match[3]));
        command = match.prefix().str() + match.suffix().str();
    }
    while (std::regex_search(command, match, regex_2))
    {
        desired_positions[std::stoi(match[1])] = std::make_pair(std::stoi(match[2]), 0);
        command = match.prefix().str() + match.suffix().str();
    }
    if (std::regex_search(command, match, regex_3))
    {
        movement_time = std::stoi(match[1]);
    }

    if (isValidSsc32uCommand(movement_time, desired_positions))
    {
        // TODO connect this somewhere
    }
}

bool JointStateConverter::isValidSsc32uCommand(
    double movement_time, const std::map<int16_t, std::pair<int16_t, int16_t>>& desired_positions) const
{
    return movement_time != std::string::npos && desired_positions.size() != 0;
}

void JointStateConverter::ssc32uCommandReceived(const robotsimulation::ssc32u_command msg)
{
    try
    {
        parseSsc32uCommand(msg);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM(e.what());
    }
}

JointStateConverter::~JointStateConverter()
{
}

// std::cout << "atdest: " << at_destination_ << " increase: " << current_increase << " pwmToGo: " << pwm_to_go
//          << "   " << current_positions_[servo_item.first] << "," << servo_item.second.first << " angle: ";
// std::cout << connected_servos_[servo_item.first].getDegreesFromPulseWidth(
//                 static_cast<int16_t>(current_positions_[servo_item.first]))
//          << std::endl;
