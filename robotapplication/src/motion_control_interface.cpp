/*
 * motion_control_interface.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: Thomas Maters
 */

#include "motion_control_interface.h"

MotionControlInterface::MotionControlInterface()
{
    target_position_subscriber_ =
        subsriber_node_.subscribe("pickupTarget", 10, &MotionControlInterface::moveToTarget, this);

    multiple_arm_pos_publisher_ =
        publisher_node_.advertise<robotarminterface::moveMultipleServosMsg>("moveMultipleServos", 10);
    defined_arm_pos_publisher_ =
        publisher_node_.advertise<robotarminterface::moveServoDefinedMsg>("moveServoToDefinedPosition", 10);
}

MotionControlInterface::~MotionControlInterface()
{
    // TODO Auto-generated destructor stub
}

void MotionControlInterface::moveToTarget(const robotapplication::pick_and_place msg)
{
}

void MotionControlInterface::publishMultipleServoMsg(const robotarminterface::moveMultipleServosMsg& msg)
{
    if (ros::ok())
    {
        multiple_arm_pos_publisher_.publish(msg);
    }
}

void MotionControlInterface::publishDefinedServoMsg(const robotarminterface::moveServoDefinedMsg& msg)
{
    if (ros::ok())
    {
        defined_arm_pos_publisher_.publish(msg);
    }
}

Matrix<3, 2, double> MotionControlInterface::getServoConfigurationSpace() const
{
    try
    {
        XmlRpc::XmlRpcValue servo_config;
        subsriber_node_.getParam("servos", servo_config);
        Matrix<3, 2, double> servo_configuration_space;

        std::size_t count = 0;
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = servo_config.begin(); it != servo_config.end();
             ++it)
        {
            if (count == servo_configuration_space.getHeight())
            {
                break;
            }
            if (it->second.size() != 10)
            {
                throw std::runtime_error("Invalid parameter when trying to load servo configuration");
            }

            if (static_cast<int>(it->second[0]) == count)
            {
                servo_configuration_space.at(count, 0) = static_cast<int>(it->second[2]);
                servo_configuration_space.at(count, 1) = static_cast<int>(it->second[3]);
                count++;
            }
        }

        return servo_configuration_space;
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    throw std::runtime_error("Something failed when getting the servo configuration from the ros param server.");
}

Matrix<3, 1, double> MotionControlInterface::getArmLenghtConfiguration() const
{
    try
    {
        XmlRpc::XmlRpcValue arm_config;
        subsriber_node_.getParam("arm_lengths", arm_config);
        Matrix<3, 1, double> arm_configuration;

        for (std::size_t i = 0; i < arm_config.size(); ++i)
        {
            arm_configuration.at(i, 0) = static_cast<double>(arm_config[i]);
        }
        arm_configuration *= 1000;

        return arm_configuration;
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    throw std::runtime_error("Something failed when getting the arm lengths configuration from the ros param server.");
}
