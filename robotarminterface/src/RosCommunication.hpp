/*
 * RosCommunication.hpp
 *
 *  Created on: Mar 1, 2017
 *      Author: thomas
 */

#ifndef BEROEPSPRODUCT_1_INTERFACE_ROSCOMMUNICATION_HPP_
#define BEROEPSPRODUCT_1_INTERFACE_ROSCOMMUNICATION_HPP_

#include "MotionControl.hpp"
#include "robotarminterface/emergencyStopMsg.h"
#include "robotarminterface/interfaceStateSrv.h"
#include "robotarminterface/moveMultipleServosMsg.h"
#include "robotarminterface/moveServo.h"
#include "robotarminterface/moveServoDefinedMsg.h"
#include "robotarminterface/moveSingleServoMsg.h"
#include "robotsimulation/ssc32u_command.h"

#include "../../shared/src/Servo.hpp"
#include "ros/ros.h"

class RosCommunication
{
  public:
    /**
     * Initializes the ros communication.
     **/
    bool init();

    /**
     * Gets an RosCommunication instance.
     **/
    static RosCommunication& getInstance();

    /**
     * Callback function for an interface state request.
     * @param req Request struct.
     * @param res Response struct.
     * @return Returns if ros can send the response back.
     **/
    bool getInterfaceStateRequest(robotarminterface::interfaceStateSrv::Request& req,
                                  robotarminterface::interfaceStateSrv::Response& res);

    /**
     * Handler when it receives an moveServoDefinedMsg message.
     * @param msg Message type.
     **/
    void moveServoToDefinedPosCallback(const robotarminterface::moveServoDefinedMsg& msg);

    /**
     * Handler when it receives an moveSingleServoMsg message.
     * @param msg Message type.
     **/
    void moveSingleServoCallback(const robotarminterface::moveSingleServoMsg& msg);

    /**
     * Handler when it receives an moveMultipleServosMsg message.
     * @param msg Message type.
     **/
    void moveMultipleServosCallback(const robotarminterface::moveMultipleServosMsg& msg);

    /**
     * Handler when it receives an emergencyStopMsg message.
     * @param msg Message type.
     **/
    void emergencyStopCallback(const robotarminterface::emergencyStopMsg& msg);

    /**
    * Sends ssc32u command to a ros topic.
    * @param command Command to send on topic.
    **/
    void sendSsc32uCommand(const std::string command);

    /**
     * Retrieves servo configuration from the ros param server.
     * @return Returns a vector with servo configurations.
     */
    std::vector<Servo> getRosParamServoConfiguration() const;

    virtual ~RosCommunication();

  private:
    RosCommunication();

    ros::NodeHandle serviceNode;
    ros::NodeHandle subsriberNode;
    ros::NodeHandle publisherNode;

    ros::ServiceServer interfaceStateService;

    ros::Subscriber moveServoToDefinedPositionSubscriber;
    ros::Subscriber moveSingleServoSubscriber;
    ros::Subscriber moveMultipleServosSubscriber;
    ros::Subscriber emergencyStopSubscriber;

    ros::Publisher ssc32uCommandoPublisher;
};

#endif /* BEROEPSPRODUCT_1_INTERFACE_ROSCOMMUNICATION_HPP_ */
