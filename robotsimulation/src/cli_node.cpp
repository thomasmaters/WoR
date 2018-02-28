/* 
 * File:   cli_node.cpp
 * Author: Thomas Maters && Peter van Leeuwen
 * 
 * Created on March 21, 2017, 3:05 PM
 */

#include "ros/ros.h"
#include "robotsimulation/ssc32u_command.h"
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
	  ros::init(argc, argv, "cli_node");

	  std::vector<std::string> positions =
	  {
			  "#0P1500#2P1450#3P2000#4P1500T4000",
			  "#0P600#1P1300#3P1600T500",
			  "#4P500T2500"
	  };

	  ros::NodeHandle n;
	  ros::Publisher chatter_pub = n.advertise<robotsimulation::ssc32u_command>("ssc32u_topic", 1000);
	  ros::Rate loop_rate(0.15);

	  std::this_thread::sleep_for(std::chrono::seconds(1));

	  int count = 0;
	  std::cout << positions.size() << std::endl;
	  while (ros::ok() && count < positions.size()) {

	    robotsimulation::ssc32u_command msg;

	    msg.command = positions.at(count);
	    std::cout << positions.at(count) << std::endl;
	    chatter_pub.publish(msg);

	    ros::spinOnce();

	    loop_rate.sleep();
	    ++count;
	  }
	return 0;
}
