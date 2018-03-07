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

std::map<uint8_t, SimulationServo> Ssc32uParserInterface::getRosParamServoConfiguration() const
{
    try
    {
        std::map<uint8_t, SimulationServo> connected_servos;
        XmlRpc::XmlRpcValue servo_config;
        XmlRpc::XmlRpcValue sim_servo_config;
        subsriber_node_.getParam("servos", servo_config);
        subsriber_node_.getParam("sim_servos", sim_servo_config);

        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = servo_config.begin(); it != servo_config.end();
             it++)
        {
            if (it->second.size() != 10)
            {
                throw std::runtime_error("Invalid parameter when trying to load servo configuration");
            }

            if (subsriber_node_.hasParam("sim_servos/" + it->first))
            {
                ROS_WARN_STREAM("Servo: '" << it->first << "' has an simulation specific conversion.");
                connected_servos.insert(std::make_pair<uint8_t, SimulationServo>(
                    static_cast<uint8_t>(static_cast<int>(it->second[0])),
                    SimulationServo(
                        static_cast<int>(it->second[0]), static_cast<std::string>(it->second[1]),
                        static_cast<int>(it->second[2]), static_cast<int>(it->second[3]),
                        static_cast<int>(it->second[4]), static_cast<int>(it->second[5]),
                        static_cast<int>(it->second[6]), static_cast<int>(it->second[7]),
                        static_cast<double>(it->second[8]), static_cast<bool>(it->second[9]),
                        static_cast<SimulationServo::JointTypes>(static_cast<int>(sim_servo_config[it->first][0])),
                        static_cast<double>(sim_servo_config[it->first][1]),
                        static_cast<double>(sim_servo_config[it->first][2]))));
            }
            else
            {
                connected_servos.insert(std::make_pair<uint8_t, SimulationServo>(
                    static_cast<uint8_t>(static_cast<int>(it->second[0])),
                    SimulationServo(static_cast<int>(it->second[0]), static_cast<std::string>(it->second[1]),
                                    static_cast<int>(it->second[2]), static_cast<int>(it->second[3]),
                                    static_cast<int>(it->second[4]), static_cast<int>(it->second[5]),
                                    static_cast<int>(it->second[6]), static_cast<int>(it->second[7]),
                                    static_cast<double>(it->second[8]), static_cast<bool>(it->second[9]))));
            }
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
