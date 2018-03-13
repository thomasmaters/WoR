/*
 * ssc32u_parser_to_jointstate.cpp
 *
 *  Created on: Mar 2, 2018
 *      Author: Thomas Maters
 */

#include "ssc32u_parser_to_jointstate.h"

#include <cmath>
#include <regex>

JointStateConverter::JointStateConverter() : cancel_movement_(false), at_destination_(true), stopping_(false)
{
    connected_servos_ = getRosParamServoConfiguration();

    joint_state_msg_.position.resize(connected_servos_.size());
    joint_state_msg_.name.push_back("base_link2turret");
    joint_state_msg_.name.push_back("turret2upperarm");
    joint_state_msg_.name.push_back("upperarm2forearm");
    joint_state_msg_.name.push_back("forearm2wrist");
    joint_state_msg_.name.push_back("wrist2hand");
    joint_state_msg_.name.push_back("gripper_left2hand");

    current_positions_[0] = 1500;
    current_positions_[1] = 1500;
    current_positions_[2] = 1500;
    current_positions_[3] = 1500;
    current_positions_[4] = 1500;
    current_positions_[5] = 1500;

    concurrent_joint_state_publish_thread_ = std::thread([this]() {
        ros::Rate rate(FPS);
        while (!stopping_)
        {
            if (at_destination_)
            {
                constructJointStateMessage();
                rate.sleep();
            }
        }
    });
}

void JointStateConverter::constructJointStateMessage()
{
    publish_mutex.lock();

    for (const std::pair<const uint8_t, double>& servo_item : current_positions_)
    {
        joint_state_msg_.position[servo_item.first] =
            connected_servos_.at(servo_item.first).getJointStateFromPulseWidth(static_cast<int16_t>(servo_item.second));
    }
    joint_state_msg_.header.stamp = ros::Time::now();
    sendJointStateMessage(joint_state_msg_);

    publish_mutex.unlock();
}

void JointStateConverter::moveToDesitnation(double movement_time,
                                            const std::map<int16_t, std::pair<int16_t, int16_t>>& desired_positions)
{
    ROS_INFO("Starting to move to destination.");
    ROS_DEBUG_STREAM("Movement_time: " << movement_time << " desired_positions size: " << desired_positions.size());
    ros::Rate rate(FPS);
    // Move slowly to the destination.
    while (movement_time > 0 && !cancel_movement_ && !stopping_)
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
            else
            {
                pwm_to_go = (double)current_positions_[servo_item.first] - (double)servo_item.second.first;
                current_increase = pwm_to_go / movement_time / FPS * -1000;
            }

            current_positions_[servo_item.first] += current_increase;
            if (current_positions_[servo_item.first] < connected_servos_.at(servo_item.first).getMinPulseWidth())
            {
                current_positions_[servo_item.first] = connected_servos_.at(servo_item.first).getMinPulseWidth();
            }
        }

        rate.sleep();

        constructJointStateMessage();
        // Decrease time left.
        movement_time -= 1000 / FPS;
    }
    at_destination_ = true;
    ROS_INFO("Reached destination");
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
        if (moving_thread_.joinable())
        {
            cancel_movement_ = true;
            moving_thread_.join();
            cancel_movement_ = false;
        }
        at_destination_ = false;
        std::thread temp = std::thread([this, movement_time, desired_positions]() {
            if (true)
            {
                moveToDesitnation(movement_time, desired_positions);
            }
        });
        moving_thread_.swap(temp);
    }
    else
    {
        ROS_WARN("Invalid movement command received.");
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
