/*
 * Cup.h
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#ifndef WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_
#define WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_

#include "virtual_cup_interface.h"

#include "robotsimulation/cup_data.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#define CUP_HEIGHT .1
#define CUP_RADIUS 0.05
#define GRAVITY 9.8     // m/s^2
#define CUP_WEIGHT 0.1  // kg
#define FPS 30

class Cup : public VirtualCupInterface
{
  public:
    enum CupState
    {
        IDLE,
        FALLING,
        GRABBED,
        PUSHED
    };

    Cup(const std::string& a_namespace);

    void loop();

    bool canFall();

    void checkForCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right);

    void applyGravity();

    virtual ~Cup();

  private:
    CupState cup_state;
    std::string namespace_;  /// Unique namespace so we can have multiple markers.

    double fall_velocity_;

    visualization_msgs::Marker marker_;
    robotsimulation::cup_data cup_data_;
};

#endif /* WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_ */
