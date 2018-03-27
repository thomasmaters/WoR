/*
 * RosCommunication.h
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#ifndef SRC_ROSCOMMUNICATION_H_
#define SRC_ROSCOMMUNICATION_H_

#include "InverseKinematics.h"
#include "robotarminterface/moveMultipleServosMsg.h"
#include "robotarminterface/moveServo.h"
#include "robotarminterface/moveServoDefinedMsg.h"
//#include "robotarminterface/moveServoMsg.h"
#include "robotapplication/PickAndPlace.h"
#include "robotapplication/arm_control_msg.h"
#include "robotapplication/arm_position.h"
#include "robotapplication/pickup_target.h"
#include "robotarminterface/moveSingleServoMsg.h"
#include "ros/ros.h"

// Official values as taken from the documentation.
// static const Matrix<3, 1, double> armLengths = Matrix<3, 1, double>({ 0, 146, 187 });
// Default values in the simulator.
static const Matrix<3, 1, double> armLengths = Matrix<3, 1, double>({ 0, 180, 200 });

class RosCommunication
{
  public:
    RosCommunication();

    /*
     *
     */
    void moveToTargetCallback(const robotapplication::PickAndPlace msg);

    /*
     *
     */
    void moveToPos(const float x, const float y, const float z, const float rotation, const bool gripperState = false);

    virtual ~RosCommunication();

  private:
    ros::NodeHandle subsriber_node_;
    ros::NodeHandle publisher_node_;
    ros::Subscriber target_position_subscriber_;
    ros::Publisher multipleArmPositions;
    ros::Publisher singleArmPosition;
    ros::Publisher definedArmPosition;

    Matrix<3, 1, double> phis;
    Matrix<3, 1, double> lastPhis;
};

#endif /* SRC_ROSCOMMUNICATION_H_ */
