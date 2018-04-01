/*
 * File:   demo_node.cpp
 * Author: Thomas Maters && Peter van Leeuwen
 *
 * Created on March 21, 2017, 3:05 PM
 */

#include <thread>
#include "robotsimulation/ssc32u_command.h"
#include "ros/ros.h"

const std::vector<std::string> positions = { "#0P1535#1P1700#2P1790#5P700T2500",
                                             "#1P1250#3P1980T2500",
                                             "#1P1190#2P1650#3P1820T2500",
                                             "#5P1400T1000",
                                             "#0P1000#1P1550#2P1345#3P1240T2500",
                                             "#5P700T1000",
                                             "#0P1500#1P1900#2P1790#3P500#5P1527T3000" };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_node");

    ros::NodeHandle n;
    ros::Publisher demo_command_publisher = n.advertise<robotsimulation::ssc32u_command>("ssc32u_topic", 1000);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    ROS_INFO("Starting demo in: 3");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_INFO("Starting demo in: 2");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_INFO("Starting demo in: 1");

    int count = 0;
    robotsimulation::ssc32u_command msg;
    ros::Rate loop_rate(0.15);
    while (ros::ok() && count < positions.size())
    {
        msg.command = positions.at(count);
        demo_command_publisher.publish(msg);
        ROS_INFO_STREAM("Publishing command: " << msg.command);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
