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
#define CUP_TO_GRIPPER_COLLISION 0.07

class Cup : public VirtualCupInterface
{
  public:
    enum CupState
    {
        IDLE,
        FALLING,
        GRABBED,
        PUSHED,
        TIPPED
    };

    enum GripperToCupState
    {
        LEFT = -1,
        RIGHT = 1,
        CENTER = 0
    };

    Cup(const std::string& a_namespace, float cup_x = 0, float cup_y = 0, float cup_z = 0);

    void loop();

    void updateCupData();

    std::string cupStateToString();

    bool canFall();

    bool checkForCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right);

    Cup::GripperToCupState getDirectionToTransform(const tf::StampedTransform& transform);

    double getRightOrLeft(const tf::Vector3& up, const tf::Vector3& forward, const tf::Vector3& from,
                          const tf::Vector3& to) const;

    void applyCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right);

    bool canBeGrabbed(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left);

    void applyGrabbing(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left);

    void applyGravity();

    double calculateSpeed(const tf::Vector3& old_pos, const tf::Vector3& new_pos);

    double calculateAcceleration(double old_speed, double new_speed);

    virtual ~Cup();

  private:
    tf::Vector3 cup_origin_;
    tf::Vector3 cup_center_;

    CupState cup_state;
    std::string namespace_;  /// Unique namespace so we can have multiple markers.

    double fall_velocity_;
    double acceleration_;
    double velocity_;

    visualization_msgs::Marker marker_;
    robotsimulation::cup_data cup_data_;

    ros::Time last_frame_time_;
};

#endif /* WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_ */
