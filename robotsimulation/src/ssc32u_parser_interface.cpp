/*
 * RosCommunication.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#include "ssc32u_parser_interface.h"

#include <map>

Ssc32uParserInterface::Ssc32uParserInterface()
{
    target_position_subscriber_ =
        subsriber_node_.subscribe("ssc32u_topic", 10, &Ssc32uParserInterface::ssc32uCommandReceived, this);
    joint_state_publisher_ = publisher_node_.advertise<sensor_msgs::JointState>("joint_states", 10);
}

void Ssc32uParserInterface::ssc32uCommandReceived(const robotsimulation::ssc32u_command msg)
{
    ROS_WARN("Callback Ssc32uParserInterface::commandReceivedCallback should be overridden in a derived class.");
}

void Ssc32uParserInterface::sendJointStateMessage(const sensor_msgs::JointState& msg) const
{
    if (ros::ok())
    {
        joint_state_publisher_.publish(msg);
        ros::spinOnce();
    }
}

std::vector<Servo> Ssc32uParserInterface::getRosParamServoConfiguration() const
{
    try
    {
        std::vector<Servo> connected_servos;
        XmlRpc::XmlRpcValue my_list;
        subsriber_node_.getParam("servos", my_list);

        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = my_list.begin(); it != my_list.end(); it++)
        {
            if (it->second.size() != 10)
            {
                throw std::runtime_error("Invalid parameter when trying to load servo configuration");
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
    throw std::runtime_error("Something failed when getting the servo configuration from the ros param server.");
}

Ssc32uParserInterface::~Ssc32uParserInterface()
{
}
