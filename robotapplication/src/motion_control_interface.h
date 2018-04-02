/*
 * motion_control_interface.h
 *
 *  Created on: Mar 28, 2018
 *      Author: Thomas Maters
 */

#ifndef WOR_ROBOTAPPLICATION_SRC_MOTION_CONTROL_INTERFACE_H_
#define WOR_ROBOTAPPLICATION_SRC_MOTION_CONTROL_INTERFACE_H_

#include "Matrix.hpp"

#include "robotapplication/pick_and_place.h"
#include "robotarminterface/moveMultipleServosMsg.h"
#include "robotarminterface/moveServo.h"
#include "robotarminterface/moveServoDefinedMsg.h"
#include "robotarminterface/moveSingleServoMsg.h"

#include "ros/ros.h"

/**
 * Interface for the motion_control_node
 */
class MotionControlInterface
{
  public:
    MotionControlInterface();
    virtual ~MotionControlInterface();

  protected:
    /**
     * Gets called when a pick_and_place message is received.
     * @param msg
     */
    virtual void moveToTarget(const robotapplication::pick_and_place msg);

    /**
     * Publishes a moveMultipleServosMsg message.
     * @param msg To publish.
     */
    void publishMultipleServoMsg(const robotarminterface::moveMultipleServosMsg& msg);

    /**
     * Publishes a moveServoDefinedMsg message.
     * @param msg To publish.
     */
    void publishDefinedServoMsg(const robotarminterface::moveServoDefinedMsg& msg);

    /**
     * Gets servo configuration from the rosparam server.
     * @return Matrix with the servo's connected to pin 0,1,2 with their min and max angles in degrees.
     */
    Matrix<3, 2, double> getServoConfigurationSpace() const;

    /**
     * Gets the arm's armlenghts from the ros param server.
     * @return Matrix with the arm lenghts.
     */
    Matrix<3, 1, double> getArmLenghtConfiguration() const;

  private:
    ros::NodeHandle subsriber_node_;
    ros::NodeHandle publisher_node_;
    ros::Subscriber target_position_subscriber_;
    ros::Publisher multiple_arm_pos_publisher_;
    ros::Publisher defined_arm_pos_publisher_;
};

#endif /* WOR_ROBOTAPPLICATION_SRC_MOTION_CONTROL_INTERFACE_H_ */
