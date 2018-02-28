/*
 * RosCommunication.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#include "RosCommunication.h"
#include <thread>
#include <regex>
#include <cmath>

#define FPS 60

RosCommunication::RosCommunication()
{
    target_position_subscriber_ =
        subsriber_node_.subscribe("ssc32u_topic", 10, &RosCommunication::commandReceivedCallback, this);
    joint_state_publisher_ = publisher_node_.advertise<sensor_msgs::JointState>("joint_states", 10);

	connected_servos_.push_back(Servo(0, "base", -90, 90, 500, 2500,-90, 90, 0.22,true));
	connected_servos_.push_back(Servo(1, "shoulder", -30, 90, 900, 1900, -30, 90, 0.19));
	connected_servos_.push_back(Servo(2, "elbow", 0, 135, 700, 1920 , 0, 135, 0.28));
	connected_servos_.push_back(Servo(3, "wrist", -90, 90, 500, 2500, -90, 90, 0.24));
	connected_servos_.push_back(Servo(4, "gripper", -1, 1  , 500, 2500, -90, 90, 0.16));
	connected_servos_.push_back(Servo(5, "wrist_rotate", -90, 90, 500, 2500, -90, 90, 0.16));

	current_positions_[0] = 1500;
	current_positions_[1] = 1500;
	current_positions_[2] = 1500;
	current_positions_[3] = 1500;
	current_positions_[4] = 1500;
	current_positions_[5] = 1500;

    std::cout << __PRETTY_FUNCTION__ << std::endl;
    ros::spin();
}

void RosCommunication::commandReceivedCallback(const robotsimulation::ssc32u_command msg)
{
	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.name.resize(connected_servos_.size());
	joint_state_msg.position.resize(connected_servos_.size());
	joint_state_msg.name[0] = "base_link2turret";
	joint_state_msg.name[1] = "turret2upperarm";
	joint_state_msg.name[2] = "upperarm2forearm";
	joint_state_msg.name[5] = "wrist2hand";
	joint_state_msg.name[4] = "gripper_left2hand";
	joint_state_msg.name[3] = "forearm2wrist";

	std::cout << msg.command << std::endl;
	std::string command = msg.command;
	bool at_destination = false;

	std::regex regex_1("#(\\d*)P(\\d*)S(\\d*)");
	std::regex regex_2("#(\\d*)P(\\d*)");
	std::regex regex_3("T(\\d*)");
	std::smatch match;
	double movement_time = std::string::npos;

	//<servoNumber, pair<toAngle, speed>>
	std::map<int16_t, std::pair<int16_t,int16_t>> desired_positions;

	//Parse command.
	while(std::regex_search(command, match, regex_1))
	{
		std::cout << "long match: " << match[1] << "," << match[2] << "," << match[3] << std::endl;
		desired_positions[std::stoi(match[1])] = std::make_pair(std::stoi(match[2]),std::stoi(match[3]));
		command = match.prefix().str() + match.suffix().str() ;
	}
    while(std::regex_search(command, match, regex_2))
    {
    	std::cout << "short match: " << match[1] << "," << match[2] << std::endl;
    	desired_positions[std::stoi(match[1])] = std::make_pair(std::stoi(match[2]),0);
    	command = match.prefix().str() + match.suffix().str() ;
    }
	if(std::regex_search(command, match, regex_3))
	{
		movement_time = std::stoi(match[1]);
	}

	//Did we parse a correct command?
	if(movement_time == std::string::npos || desired_positions.size() == 0)
	{
		return;
	}

	std::cout << "desiredpos size: " << desired_positions.size() << std::endl;

	ros::Rate rate(FPS);
	//Move slowly to the destination.
	while(!at_destination)
	{
		at_destination = true;
		for(const std::pair<const int16_t,std::pair<int16_t,int16_t>> &servo_item : desired_positions)
		{
			double current_increase = 0;
			double pwm_to_go = 0;
			if(servo_item.second.second != 0)
			{
				pwm_to_go = std::floor((double)current_positions_[servo_item.first] - (double)servo_item.second.first);

				if(abs(pwm_to_go) > servo_item.second.second / FPS)
				{
					current_increase = servo_item.second.second / FPS * (pwm_to_go < 0 ? 1 : -1);
				}
				else
				{
					current_increase = -pwm_to_go;
				}
			}
			else if(movement_time > 0) //Do we still have time left
			{
				pwm_to_go = (double)current_positions_[servo_item.first] - (double)servo_item.second.first;
				current_increase = pwm_to_go / movement_time / FPS * -1000;
			}

			if(pwm_to_go != 0)
			{
				at_destination = false;
			}

			current_positions_[servo_item.first] += current_increase;
			if(current_positions_[servo_item.first] < connected_servos_[servo_item.first].min_pulse_width_)
			{
				current_positions_[servo_item.first] = connected_servos_[servo_item.first].min_pulse_width_;
			}
			std::cout << "atdest: " << at_destination << " increase: " << current_increase << " pwmToGo: " << pwm_to_go << "   " << current_positions_[servo_item.first] << "," << servo_item.second.first << " angle: ";
			std::cout << connected_servos_[servo_item.first].getDegreesFromPulseWidth(static_cast<int16_t>(current_positions_[servo_item.first])) << std::endl;
		}
		//Decrease timeleft.
		movement_time -= 1000/FPS;

		for(const std::pair<const uint8_t,double>& servo_item : current_positions_)
		{

			joint_state_msg.position[servo_item.first] = (connected_servos_[servo_item.first].getDegreesFromPulseWidth(static_cast<int16_t>(servo_item.second)) * M_PI) / 180;
		}
		joint_state_msg.header.stamp = ros::Time::now();
		joint_state_publisher_.publish(joint_state_msg);

		ros::spinOnce();
		if(at_destination)
		{
			break;
		}
		rate.sleep();
	}
}

RosCommunication::~RosCommunication()
{
    // TODO Auto-generated destructor stub
}
