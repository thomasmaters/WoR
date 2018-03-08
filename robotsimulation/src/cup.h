/*
 * Cup.h
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#ifndef WOR_ROBOTSIMULATION_SRC_CUP_H_
#define WOR_ROBOTSIMULATION_SRC_CUP_H_

#include <visualization_msgs/Marker.h>

#define CUP_HEIGHT .1
#define CUP_RADIUS 0.05

class Cup
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

    void applyGravity();

    virtual ~Cup();

  private:
    CupState cup_state;
    std::string namespace_;  /// Unique namespace so we can have multiple markers.
    tf::TransformListener listener_;
    visualization_msgs::Marker marker_;
};

#endif /* WOR_ROBOTSIMULATION_SRC_CUP_H_ */
